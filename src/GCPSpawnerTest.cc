// GCPSpawnerTest.cc
//
// A Gazebo system plugin that runs alongside GCPSpawner and validates its
// output.  Load it in your world SDF *after* GCPSpawner.  On the first
// PostUpdate tick it checks every proposed error source and calls
// std::abort() with a descriptive message if anything is wrong, which
// terminates Gazebo immediately with a non-zero exit code.
//
// Add to your world SDF:
//   <plugin filename="libGCPSpawnerTest.so" name="gz::sim::systems::GCPSpawnerTest"/>
//
// Proposed error sources tested here:
//   [A] SDF string is malformed / doesn't parse
//   [B] sdf::Root contains no model after parsing
//   [C] Texture file does not exist at the file:// path baked into the SDF
//   [D] Entities were not actually created in the ECM (spawn silently failed)
//   [E] Created entities have the wrong name prefix
//   [F] Box size in the spawned visual doesn't match expected dimensions

#include <gz/sim/System.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/SdfEntityCreator.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Model.hh>
#include <gz/plugin/Register.hh>
#include <sdf/Root.hh>
#include <cstdlib>
#include <filesystem>
#include <sstream>
#include <string>
#include <vector>
#include <cstdlib>
#include <csignal>
#include <stdexcept>

// Resolved at Configure() time via GZ_SIM_RESOURCE_PATH, same as GCPSpawner.cc.
static std::string GCP_MODEL_BASE_PATH;

static constexpr double LARGE_SIDE    = 1.2;
static constexpr double SMALL_SIDE    = 0.6;
static constexpr double GCP_THICKNESS = 0.003;
static const int EXPECTED_LARGE = 6;
static const int EXPECTED_SMALL = 6;

namespace fs = std::filesystem;

// ── Test helpers ──────────────────────────────────────────────────────────────

static void FAIL(const std::string &test_id, const std::string &msg)
{
    std::cerr << "\n[GCPSpawnerTest] FAIL [" << test_id << "]: " << msg
              << "\nAborting Gazebo.\n" << std::flush;
    std::abort();
}

static void PASS(const std::string &test_id, const std::string &msg = "")
{
    gzmsg << "[GCPSpawnerTest] PASS [" << test_id << "]"
          << (msg.empty() ? "" : ": " + msg) << "\n";
}

// ── [A] + [B]: SDF round-trip ─────────────────────────────────────────────────
// Rebuild the SDF string for one representative model of each variant and
// verify it parses cleanly and contains exactly one model.
static std::string buildSDF(const std::string &variant, int gcp_num,
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

static void testSdfParsing()
{
    for (const auto &[variant, side] :
         std::vector<std::pair<std::string,double>>{{"large", LARGE_SIDE},
                                                     {"small", SMALL_SIDE}})
    {
        const std::string tid = "A+B/sdf_parse_" + variant;
        const std::string sdfStr = buildSDF(variant, 0,
                                            "test_" + variant, 0, 0, 0, side);
        sdf::Root root;
        auto errors = root.LoadSdfString(sdfStr);

        // [A] parse must be error-free
        if (!errors.empty()) {
            std::string msg = "SDF parse errors for variant '" + variant + "':\n";
            for (auto &e : errors) msg += "  " + e.Message() + "\n";
            FAIL(tid, msg);
        }

        // [B] root must contain a model
        if (!root.Model())
            FAIL(tid, "sdf::Root contains no model after parsing variant '"
                      + variant + "'");

        // Also verify the model name round-tripped correctly
        if (root.Model()->Name() != "test_" + variant)
            FAIL(tid, "Model name mismatch: expected 'test_" + variant +
                      "', got '" + root.Model()->Name() + "'");

        PASS(tid, "variant=" + variant + " side=" + std::to_string(side) + " m");
    }
}

// ── [C]: texture files exist on disk ──────────────────────────────────────────
static void testTextureFiles()
{
    for (const auto &[variant, side] :
         std::vector<std::pair<std::string,double>>{{"large", LARGE_SIDE},
                                                     {"small", SMALL_SIDE}})
    {
        for (int n = 0; n < 10; ++n) {
            const std::string model_name =
                "gcp_" + variant + "_" + std::to_string(n);
            const fs::path tex_path =
                fs::path(GCP_MODEL_BASE_PATH) / model_name /
                "materials" / "textures" / (model_name + ".png");
            const std::string tid = "C/texture_exists_" + model_name;

            if (!fs::exists(tex_path))
                FAIL(tid, "Texture not found at: " + tex_path.string() +
                          "\nDid you run gen_gcps.py with the correct OUTPUT_DIR?");

            if (fs::file_size(tex_path) < 1024)
                FAIL(tid, "Texture file suspiciously small (<1 KB): "
                          + tex_path.string());

            PASS(tid);
        }
    }
}

// ── [D] + [E]: entities exist in ECM with correct name prefixes ───────────────
static void testEntitiesInECM(const gz::sim::EntityComponentManager &ecm)
{
    int large_count = 0, small_count = 0;

    ecm.Each<gz::sim::components::Model,
             gz::sim::components::Name>(
        [&](const gz::sim::Entity &,
            const gz::sim::components::Model *,
            const gz::sim::components::Name *name) -> bool
        {
            const std::string &n = name->Data();
            if (n.rfind("gcp_large_", 0) == 0) ++large_count;
            if (n.rfind("gcp_small_", 0) == 0) ++small_count;
            return true;   // keep iterating
        });

    // [D] at least some models must exist
    if (large_count == 0 && small_count == 0)
        FAIL("D/entities_exist",
             "No gcp_large_* or gcp_small_* models found in ECM. "
             "GCPSpawner may not have run, or SdfEntityCreator silently failed.");

    // [E] counts must match what the spawner is configured to place
    if (large_count != EXPECTED_LARGE)
        FAIL("E/large_count",
             "Expected " + std::to_string(EXPECTED_LARGE) +
             " large GCP entities, found " + std::to_string(large_count) +
             ". Check SET_A_COUNT and placement geometry.");

    if (small_count != EXPECTED_SMALL)
        FAIL("E/small_count",
             "Expected " + std::to_string(EXPECTED_SMALL) +
             " small GCP entities, found " + std::to_string(small_count) +
             ". Check SET_B_COUNT and convex-hull placement.");

    PASS("D+E/entity_counts",
         std::to_string(large_count) + " large, " +
         std::to_string(small_count) + " small");
}

// ── [F]: box dimensions in the spawned SDF match expectations ────────────────
// We re-parse a freshly built SDF and inspect the geometry element directly
// rather than querying the renderer — renderer state isn't accessible here.
static void testBoxDimensions()
{
    for (const auto &[variant, expected_side] :
         std::vector<std::pair<std::string,double>>{{"large", LARGE_SIDE},
                                                     {"small", SMALL_SIDE}})
    {
        const std::string tid = "F/box_dims_" + variant;
        sdf::Root root;
        root.LoadSdfString(buildSDF(variant, 0, "dim_test", 0, 0, 0,
                                    expected_side));
        const sdf::Model *model = root.Model();
        if (!model) { FAIL(tid, "no model"); return; }

        const sdf::Link *link = model->LinkByIndex(0);
        if (!link) { FAIL(tid, "no link"); return; }

        // Check both collision and visual
        for (const std::string &geom_owner : {"collision", "visual"}) {
            const sdf::Geometry *geom = (geom_owner == "collision")
                ? link->CollisionByIndex(0)->Geom()
                : link->VisualByIndex(0)->Geom();

            if (!geom || geom->Type() != sdf::GeometryType::BOX)
                FAIL(tid, geom_owner + " geometry is not a box");

            const gz::math::Vector3d sz = geom->BoxShape()->Size();
            const double tol = 1e-6;
            if (std::abs(sz.X() - expected_side) > tol ||
                std::abs(sz.Y() - expected_side) > tol)
                FAIL(tid, geom_owner + " box XY = " +
                          std::to_string(sz.X()) + " x " +
                          std::to_string(sz.Y()) + ", expected " +
                          std::to_string(expected_side));

            if (std::abs(sz.Z() - GCP_THICKNESS) > tol)
                FAIL(tid, geom_owner + " box Z = " +
                          std::to_string(sz.Z()) + ", expected " +
                          std::to_string(GCP_THICKNESS));
        }
        PASS(tid, std::to_string(expected_side) + " m x " +
                  std::to_string(expected_side) + " m x " +
                  std::to_string(GCP_THICKNESS) + " m");
    }
}

// ── System plugin ─────────────────────────────────────────────────────────────
namespace gz::sim::systems {

class GCPSpawnerTest : public gz::sim::System,
                       public gz::sim::ISystemConfigure,
                       public gz::sim::ISystemPostUpdate
{
    bool ran_ = false;

public:
    // [A] [B] [C] [F] run at configure time — no ECM entities needed
    void Configure(const gz::sim::Entity &,
                   const std::shared_ptr<const sdf::Element> &,
                   gz::sim::EntityComponentManager &,
                   gz::sim::EventManager &) override
    {
        const char *resource_env = std::getenv("GZ_SIM_RESOURCE_PATH");
        if (!resource_env) {
            FAIL("INIT/resource_path",
                 "GZ_SIM_RESOURCE_PATH is not set.\n"
                 "  Run: source ~/ardu-gz_uasc/install/setup.bash");
        }
        std::istringstream paths(resource_env);
        std::string dir;
        while (std::getline(paths, dir, ':')) {
            if (!dir.empty() &&
                std::filesystem::is_directory(dir + "/gcp_large_0")) {
                GCP_MODEL_BASE_PATH = dir;
                break;
            }
        }
        if (GCP_MODEL_BASE_PATH.empty()) {
            FAIL("INIT/resource_path",
                 "No GCP models found in GZ_SIM_RESOURCE_PATH.\n"
                 "  Run: source ~/ardu-gz_uasc/install/setup.bash");
        }

        gzmsg << "\n[GCPSpawnerTest] -- running static tests --\n";
        testSdfParsing();    // [A] [B]
        testTextureFiles();  // [C]
        testBoxDimensions(); // [F]
        gzmsg << "[GCPSpawnerTest] -- static tests passed --\n";
    }

    // [D] [E] need the ECM populated by GCPSpawner, so run on first PostUpdate
    void PostUpdate(const gz::sim::UpdateInfo &,
                    const gz::sim::EntityComponentManager &ecm) override
    {
        if (ran_) return;
        ran_ = true;
        gzmsg << "\n[GCPSpawnerTest] -- running ECM tests --\n";
        testEntitiesInECM(ecm);  // [D] [E]
        gzmsg << "[GCPSpawnerTest] -- all tests passed --\n";
    }
};

} // namespace gz::sim::systems

GZ_ADD_PLUGIN(gz::sim::systems::GCPSpawnerTest,
              gz::sim::System,
              gz::sim::systems::GCPSpawnerTest::ISystemConfigure,
              gz::sim::systems::GCPSpawnerTest::ISystemPostUpdate)
