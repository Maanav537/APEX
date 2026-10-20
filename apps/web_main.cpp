// Web front end: serves web/index.html and runs preset scenarios on request.
// Each run steps the real Simulation to the end and returns the whole
// trajectory as JSON; the page replays it.
//
// Options:
//   --port N   listen on localhost:N (default 8080)

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

#include <httplib.h>

#include <apex/sim/capture.hpp>
#include <apex/sim/scenarios.hpp>

namespace {

// Appends printf-style text to `out`.
template <typename... Args>
void append(std::string& out, const char* fmt, Args... args) {
    char buf[256];
    const int n = std::snprintf(buf, sizeof buf, fmt, args...);
    out.append(buf, static_cast<std::size_t>(n));
}

// Ids, names, and descriptions are fixed ASCII without quotes or backslashes.
void append_vec(std::string& out, const apex::Vec3& v) {
    append(out, "[%.3f,%.3f,%.3f]", v.x(), v.y(), v.z());
}

std::string scenarios_json() {
    std::string out = "[";
    bool first = true;
    for (const apex::ScenarioInfo& s : apex::scenario_list()) {
        out += first ? "" : ",";
        first = false;
        out += std::string("{\"id\":\"") + s.id + "\",\"name\":\"" + s.name +
               "\",\"description\":\"" + s.description + "\"}";
    }
    return out + "]";
}

// Runs the scenario to the end. Frames every guidance update (100 Hz):
// [t, mouse xyz, cat xyz, mouse speed, cat speed, range, mouse dodging, cat sees weave].
std::string run_json(const apex::Scenario& sc) {
    apex::Simulation sim = apex::make_simulation(sc);
    const int max_steps =
        static_cast<int>(std::lround(apex::scenario_max_time(sc) / apex::sim_dt));

    std::string out;
    out.reserve(1 << 20);
    out += "{\"id\":\"" + sc.id + "\",\"goal\":";
    append_vec(out, sc.goal);
    append(out, ",\"goal_radius\":%.3f,\"capture_radius\":%.3f", apex::goal_radius,
           apex::capture_radius);
    append(out, ",\"mouse_speed\":%.3f,\"cat_speed\":%.3f,\"frames\":[", sc.mouse_speed,
           sc.cat_speed);

    auto frame = [&](bool first) {
        const apex::RigidBodyState& m = sim.mouse();
        const apex::RigidBodyState& c = sim.cat();
        append(out, "%s[%.3f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%d]",
               first ? "" : ",", sim.time(), m.position.x(), m.position.y(), m.position.z(),
               c.position.x(), c.position.y(), c.position.z(), m.velocity.norm(),
               c.velocity.norm(), (m.position - c.position).norm(), sim.mouse_dodging() ? 1 : 0,
               sim.cat_sees_weave() ? 1 : 0);
    };

    auto running = [&] { return !sim.ground_hit() && !sim.captured() && !sim.reached_goal(); };
    frame(true);
    for (int i = 0; i < max_steps && running(); ++i) {
        sim.step();
        if ((i + 1) % apex::guidance_every == 0 || !running()) {
            frame(false);
        }
    }

    const char* outcome = sim.captured()       ? "captured"
                          : sim.reached_goal() ? "goal"
                          : sim.ground_hit()   ? "ground"
                                               : "timeout";
    append(out, "],\"outcome\":\"%s\",\"end_time\":%.3f}", outcome, sim.time());
    return out;
}

std::string read_file(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

}  // namespace

int main(int argc, char** argv) {
    int port = 8080;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--port" && i + 1 < argc) {
            port = std::stoi(argv[++i]);
        } else {
            std::fprintf(stderr, "usage: apex_web [--port N]\n");
            return 1;
        }
    }

    httplib::Server server;

    // Read on every request so page edits show up on reload.
    server.Get("/", [](const httplib::Request&, httplib::Response& res) {
        const std::string page = read_file(std::string(APEX_WEB_DIR) + "/index.html");
        if (page.empty()) {
            res.status = 500;
            res.set_content("web/index.html not found", "text/plain");
            return;
        }
        res.set_content(page, "text/html; charset=utf-8");
    });

    server.Get("/api/scenarios", [](const httplib::Request&, httplib::Response& res) {
        res.set_content(scenarios_json(), "application/json");
    });

    server.Get("/api/run", [](const httplib::Request& req, httplib::Response& res) {
        try {
            const std::string id = req.has_param("id") ? req.get_param_value("id") : "basic";
            const auto seed = static_cast<std::uint32_t>(
                req.has_param("seed") ? std::stoul(req.get_param_value("seed")) : 1);
            const bool vary_noise =
                req.has_param("vary_noise") && req.get_param_value("vary_noise") == "1";

            auto sc = apex::make_scenario(id, seed, vary_noise);
            if (!sc) {
                res.status = 400;
                res.set_content("{\"error\":\"unknown scenario\"}", "application/json");
                return;
            }
            // Speed overrides, kept in a sane range so a run always ends quickly.
            auto speed = [&](const char* key, double& value) {
                if (req.has_param(key)) {
                    const double v = std::stod(req.get_param_value(key));
                    if (!(v >= 5.0 && v <= 500.0)) {
                        throw std::invalid_argument("speed out of range");
                    }
                    value = v;
                }
            };
            speed("mouse_speed", sc->mouse_speed);
            speed("cat_speed", sc->cat_speed);

            res.set_content(run_json(*sc), "application/json");
        } catch (const std::exception&) {
            res.status = 400;
            res.set_content("{\"error\":\"bad parameter (speeds must be 5-500 m/s)\"}",
                            "application/json");
        }
    });

    std::printf("APEX web: open http://localhost:%d\n", port);
    std::fflush(stdout);
    if (!server.listen("127.0.0.1", port)) {
        std::fprintf(stderr, "could not listen on port %d\n", port);
        return 1;
    }
}
