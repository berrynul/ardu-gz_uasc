#include <gz/sim/System.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/components/Name.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/entity_factory.pb.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <random>
#include <vector>
#include <cmath>
#include <string>
#include <sstream>
#include <stdexcept>

// Resolved once in Configure() via the ament index.
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
    const int    SET_A_COUNT   = 6;
    const int    SET_B_COUNT   = 6;
    const double SET_A_SPACING = 0.8;
    const double SET_B_SPACING = 0.5;

    std::string worldName_;
    bool        spawned_ = false;

    // ── SDF builder ───────────────────────────────────────────────────────────
    std::string makeSDF(const std::string &variant, int gcp_num,
                        const std::string &instance_name,
                        double x, double y, double yaw, double side)
    {
        const double half_t = GCP_THICKNESS / 2.0;
        const std::string model_name =
            "gcp_" + variant + "_" + std::to_string(gcp_num);
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
    std::vector<Point2D> placeInTriangle(int count, double min_spacing,
                                          std::default_random_engine &rng) {
        double xmn=std::min({TRI_A.x,TRI_B.x,TRI_C.x});
        double xmx=std::max({TRI_A.x,TRI_B.x,TRI_C.x});
        double ymn=std::min({TRI_A.y,TRI_B.y,TRI_C.y});
        double ymx=std::max({TRI_A.y,TRI_B.y,TRI_C.y});
        std::uniform_real_distribution<double> rx(xmn,xmx), ry(ymn,ymx);
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
        if((int)placed.size()<count)
            gzwarn<<"GCPSpawner: only placed "<<placed.size()<<"/"<<count<<" large GCPs\n";
        return placed;
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
        std::vector<Point2D> placed;
        int attempts=0;
        while((int)placed.size()<count && attempts<count*300) {
            Point2D p={rx(rng),ry(rng)};
            if(!pointInConvexHull(hull,p)){attempts++;continue;}
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
        try {
            GCP_MODEL_BASE_PATH =
                ament_index_cpp::get_package_share_directory("ardupilot_gazebo")
                + "/models";
        } catch (const std::exception &e) {
            gzerr << "GCPSpawner: could not resolve package share directory: "
                  << e.what() << "\n"
                  << "  Run: source ~/ardupilot_gazebo/install/setup.bash\n";
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
        std::uniform_int_distribution<int> gcp_id(0, 9);
        std::uniform_real_distribution<double> yaw_dist(0.0, M_PI * 2.0);

        // ── Large GCPs (1.2 m) placed within triangle ─────────────────────────
        auto pos_a = placeInTriangle(SET_A_COUNT, SET_A_SPACING, rng);
        for (int i = 0; i < (int)pos_a.size(); i++) {
            req.Clear();
            req.set_sdf(makeSDF("large", gcp_id(rng),
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
        auto hull  = convexHull(pos_a);
        auto pos_b = placeInHull(SET_B_COUNT, SET_B_SPACING, hull, rng);
        for (int i = 0; i < (int)pos_b.size(); i++) {
            req.Clear();
            req.set_sdf(makeSDF("small", gcp_id(rng),
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
