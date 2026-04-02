#include <gz/sim/System.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/components/Name.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/entity_factory.pb.h>
#include <random>
#include <vector>
#include <cmath>
#include <string>
#include <sstream>
#include <cstdlib>
#include <filesystem>

// Resolved once in Configure() via GZ_SIM_RESOURCE_PATH.
static std::string GCP_MODEL_BASE_PATH;

namespace gz::sim::systems {

// ── Geometry helpers ──────────────────────────────────────────────────────────
struct Point2D { double x, y; };

double cross(Point2D a, Point2D b, Point2D c) {
    return (b.x-a.x)*(c.y-a.y) - (b.y-a.y)*(c.x-a.x);
}
bool pointInTriangle(Point2D p, Point2D a, Point2D b, Point2D c) {
    double d1=cross(a,b,p), d2=cross(b,c,p), d3=cross(c,a,p);
    return !((d1<0||d2<0||d3<0) && (d1>0||d2>0||d3>0));
}
std::vector<Point2D> convexHull(std::vector<Point2D> pts) {
    int n=pts.size(); if(n<3) return pts;
    int l=0;
    for(int i=1;i<n;i++) if(pts[i].x<pts[l].x) l=i;
    std::vector<Point2D> hull; int p=l;
    do {
        hull.push_back(pts[p]);
        int q=(p+1)%n;
        for(int i=0;i<n;i++) if(cross(pts[p],pts[i],pts[q])>0) q=i;
        p=q;
    } while(p!=l);
    return hull;
}
bool pointInConvexHull(const std::vector<Point2D>& hull, Point2D p) {
    int n=hull.size();
    for(int i=0;i<n;i++)
        if(cross(hull[i],hull[(i+1)%n],p)<0) return false;
    return true;
}
double dist(Point2D a, Point2D b) {
    return std::sqrt((a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y));
}

// ── Plugin ────────────────────────────────────────────────────────────────────
class GCPSpawner : public System,
                   public ISystemConfigure,
                   public ISystemPreUpdate
{
    static constexpr double LARGE_SIDE    = 1.2;
    static constexpr double SMALL_SIDE    = 0.6;
    static constexpr double GCP_THICKNESS = 0.003;

    const Point2D TRI_A = {-5.0, -5.0};
    const Point2D TRI_B = { 5.0, -5.0};
    const Point2D TRI_C = { 0.0,  5.0};
    const int    SET_A_MIN     = 3;
    const int    SET_A_MAX     = 4;
    const int    SET_B_MIN     = 5;
    const int    SET_B_MAX     = 10;
    const double SET_A_SPACING = 3.0;
    const double SET_B_SPACING = 0.6;

    std::string worldName_;
    bool        spawned_ = false;

    // ── SDF builder ───────────────────────────────────────────────────────────
    std::string makeSDF(const std::string &model_name,
                        const std::string &instance_name,
                        double x, double y, double yaw, double side)
    {
        const double half_t = GCP_THICKNESS / 2.0;
        const std::string tex_uri =
            "file://" + GCP_MODEL_BASE_PATH + "/" + model_name +
            "/materials/textures/" + model_name + ".png";

        std::ostringstream ss;
        ss << "<sdf version='1.9'>"
           << "<model name='" << instance_name << "'>"
           << "  <static>true</static>"
           << "  <pose>" << x << " " << y << " " << half_t
                         << " 0 0 " << yaw << "</pose>"
           << "  <link name='base'>"
           << "    <collision name='col'>"
           << "      <geometry><box><size>"
                     << side << " " << side << " " << GCP_THICKNESS
                     << "</size></box></geometry>"
           << "    </collision>"
           << "    <visual name='vis'>"
           << "      <geometry><box><size>"
                     << side << " " << side << " " << GCP_THICKNESS
                     << "</size></box></geometry>"
           << "      <material>"
           << "        <diffuse>1 1 1 1</diffuse>"
           << "        <specular>0.05 0.05 0.05 1</specular>"
           << "        <pbr><metal>"
           << "          <albedo_map>" << tex_uri << "</albedo_map>"
           << "          <metalness>0.0</metalness>"
           << "          <roughness>1.0</roughness>"
           << "        </metal></pbr>"
           << "      </material>"
           << "    </visual>"
           << "  </link>"
           << "</model></sdf>";
        return ss.str();
    }

    // ── Placement helpers ─────────────────────────────────────────────────────
    static double hullArea(const std::vector<Point2D> &hull) {
        double a = 0;
        int n = hull.size();
        for (int i = 0; i < n; i++) {
            int j = (i + 1) % n;
            a += hull[i].x * hull[j].y;
            a -= hull[j].x * hull[i].y;
        }
        return std::abs(a) / 2.0;
    }

    static constexpr double MIN_HULL_AREA = 4.0;  // m², reject degenerate layouts
    static constexpr int    MAX_RETRIES   = 50;

    std::vector<Point2D> placeInTriangle(int count, double min_spacing,
                                          std::default_random_engine &rng) {
        double xmn=std::min({TRI_A.x,TRI_B.x,TRI_C.x});
        double xmx=std::max({TRI_A.x,TRI_B.x,TRI_C.x});
        double ymn=std::min({TRI_A.y,TRI_B.y,TRI_C.y});
        double ymx=std::max({TRI_A.y,TRI_B.y,TRI_C.y});
        std::uniform_real_distribution<double> rx(xmn,xmx), ry(ymn,ymx);

        for (int retry = 0; retry < MAX_RETRIES; retry++) {
            std::vector<Point2D> placed;
            int attempts=0;
            while((int)placed.size()<count && attempts<count*300) {
                Point2D p={rx(rng),ry(rng)};
                if(!pointInTriangle(p,TRI_A,TRI_B,TRI_C)){attempts++;continue;}
                bool ok=true;
                for(auto &e:placed) if(dist(p,e)<min_spacing){ok=false;break;}
                if(ok) placed.push_back(p);
                attempts++;
            }
            if ((int)placed.size() >= count) {
                auto hull = convexHull(placed);
                if (hull.size() >= 3 && hullArea(hull) >= MIN_HULL_AREA)
                    return placed;
            }
        }
        gzwarn << "GCPSpawner: could not place " << count
               << " large GCPs with hull area >= " << MIN_HULL_AREA << " m²\n";
        return placeInTriangle(count, min_spacing * 0.5, rng);
    }

    // Signed distance from point to the nearest edge of a convex hull.
    // Positive = inside, negative = outside.
    static double distToHullEdge(const std::vector<Point2D> &hull, Point2D p) {
        double min_d = std::numeric_limits<double>::max();
        int n = hull.size();
        for (int i = 0; i < n; i++) {
            int j = (i + 1) % n;
            Point2D a = hull[i], b = hull[j];
            // Signed distance from edge (positive = inside for CCW hull)
            double len = dist(a, b);
            double d = cross(a, b, p) / len;
            min_d = std::min(min_d, d);
        }
        return min_d;
    }

    std::vector<Point2D> placeInHull(int count, double min_spacing,
                                      const std::vector<Point2D> &hull,
                                      std::default_random_engine &rng) {
        double xmn=hull[0].x,xmx=hull[0].x,ymn=hull[0].y,ymx=hull[0].y;
        for(auto &p:hull){
            xmn=std::min(xmn,p.x);xmx=std::max(xmx,p.x);
            ymn=std::min(ymn,p.y);ymx=std::max(ymx,p.y);
        }
        std::uniform_real_distribution<double> rx(xmn,xmx), ry(ymn,ymx);

        // Inset from edges by half the small GCP side so they sit fully inside
        const double edge_inset = SMALL_SIDE / 2.0;

        std::vector<Point2D> placed;
        int attempts=0;
        while((int)placed.size()<count && attempts<count*300) {
            Point2D p={rx(rng),ry(rng)};
            if(distToHullEdge(hull, p) < edge_inset){attempts++;continue;}
            bool ok=true;
            for(auto &e:placed) if(dist(p,e)<min_spacing){ok=false;break;}
            if(ok) placed.push_back(p);
            attempts++;
        }
        if((int)placed.size()<count)
            gzwarn<<"GCPSpawner: only placed "<<placed.size()<<"/"<<count<<" small GCPs\n";
        return placed;
    }

public:
    // ── Configure: resolve paths and save world name only ─────────────────────
    // Do NOT call any transport service here — UserCommands (which owns
    // /world/.../create) has not been loaded yet at Configure() time.
    // All spawning is deferred to the first PreUpdate() tick.
    void Configure(const Entity &entity,
                   const std::shared_ptr<const sdf::Element> & /*sdf*/,
                   EntityComponentManager &ecm,
                   EventManager & /*eventMgr*/) override
    {
        const char *resource_env = std::getenv("GZ_SIM_RESOURCE_PATH");
        if (!resource_env) {
            gzerr << "GCPSpawner: GZ_SIM_RESOURCE_PATH is not set.\n"
                  << "  Run: source ~/ardu-gz_uasc/install/setup.bash\n";
            return;
        }
        std::istringstream paths(resource_env);
        std::string dir;
        while (std::getline(paths, dir, ':')) {
            if (!dir.empty() &&
                std::filesystem::is_directory(dir + "/gcp_large")) {
                GCP_MODEL_BASE_PATH = dir;
                break;
            }
        }
        if (GCP_MODEL_BASE_PATH.empty()) {
            gzerr << "GCPSpawner: no GCP models found in GZ_SIM_RESOURCE_PATH.\n"
                  << "  Run: source ~/ardu-gz_uasc/install/setup.bash\n";
            return;
        }

        auto name = ecm.Component<components::Name>(entity);
        if (!name) {
            gzerr << "GCPSpawner: could not get world name\n";
            return;
        }
        worldName_ = name->Data();
        gzmsg << "GCPSpawner: configured for world [" << worldName_ << "]"
              << "  models at [" << GCP_MODEL_BASE_PATH << "]\n";
    }

    // ── PreUpdate: spawn on the very first tick ───────────────────────────────
    // By this point UserCommands is fully loaded and /world/.../create is live.
    // The spawned_ guard ensures this runs exactly once.
    void PreUpdate(const UpdateInfo & /*info*/,
                   EntityComponentManager & /*ecm*/) override
    {
        if (spawned_ || worldName_.empty()) return;
        spawned_ = true;

        const std::string serviceName = "/world/" + worldName_ + "/create";
        gz::transport::Node node;
        gz::msgs::EntityFactory req;
        gz::msgs::Boolean res;
        bool result = false;

        std::default_random_engine rng(std::random_device{}());
        std::uniform_real_distribution<double> yaw_dist(0.0, M_PI * 2.0);
        std::uniform_int_distribution<int> large_count(SET_A_MIN, SET_A_MAX);
        std::uniform_int_distribution<int> small_count(SET_B_MIN, SET_B_MAX);

        // ── Large GCPs (1.2 m) placed within triangle ─────────────────────────
        // All large GCPs use the single "gcp_large" model (no number).
        auto pos_a = placeInTriangle(large_count(rng), SET_A_SPACING, rng);
        for (int i = 0; i < (int)pos_a.size(); i++) {
            req.Clear();
            req.set_sdf(makeSDF("gcp_large",
                                "gcp_large_" + std::to_string(i),
                                pos_a[i].x, pos_a[i].y,
                                yaw_dist(rng), LARGE_SIDE));
            node.Request(serviceName, req, 2000, res, result);
            if (!result)
                gzerr << "GCPSpawner: failed to spawn gcp_large_" << i << "\n";
        }

        if ((int)pos_a.size() < 3) {
            gzerr << "GCPSpawner: need >=3 large GCPs to form hull\n";
            return;
        }

        // ── Small GCPs (0.6 m) inside convex hull of large GCPs ──────────────
        // Each small GCP gets a unique number (0..N-1) matching gcp_small_{n}.
        auto hull  = convexHull(pos_a);
        int  n_small = small_count(rng);
        auto pos_b = placeInHull(n_small, SET_B_SPACING, hull, rng);
        for (int i = 0; i < (int)pos_b.size(); i++) {
            req.Clear();
            req.set_sdf(makeSDF("gcp_small_" + std::to_string(i),
                                "gcp_small_" + std::to_string(i),
                                pos_b[i].x, pos_b[i].y,
                                yaw_dist(rng), SMALL_SIDE));
            node.Request(serviceName, req, 2000, res, result);
            if (!result)
                gzerr << "GCPSpawner: failed to spawn gcp_small_" << i << "\n";
        }

        gzmsg << "GCPSpawner: spawned " << pos_a.size()
              << " large (1.2 m) and " << pos_b.size()
              << " small (0.6 m) GCPs\n";
    }
};

} // namespace gz::sim::systems

GZ_ADD_PLUGIN(gz::sim::systems::GCPSpawner,
              gz::sim::System,
              gz::sim::systems::GCPSpawner::ISystemConfigure,
              gz::sim::systems::GCPSpawner::ISystemPreUpdate)
