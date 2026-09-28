#include "profile.h"

#include "camera_math.h"

#include <algorithm>
#include <cctype>
#include <charconv>
#include <cmath>
#include <cstring>
#include <fstream>
#include <limits>
#include <map>
#include <sstream>

using reshade::api::shader_stage;

namespace lidar {
namespace {

// ---- TOML subset ----------------------------------------------------------------------------

struct Value {
    enum Type { String, Int, Float, Bool } type = String;
    std::string s;
    int64_t i = 0;
    double f = 0;
    bool b = false;
    int line = 0;
};
using Table = std::map<std::string, Value>;

std::string_view trim(std::string_view v) {
    while (!v.empty() && (v.front() == ' ' || v.front() == '\t' || v.front() == '\r')) v.remove_prefix(1);
    while (!v.empty() && (v.back() == ' ' || v.back() == '\t' || v.back() == '\r')) v.remove_suffix(1);
    return v;
}

// Cuts a # comment that isn't inside a string.
std::string_view strip_comment(std::string_view line) {
    char quote = 0;
    for (size_t i = 0; i < line.size(); ++i) {
        const char c = line[i];
        if (quote) {
            if (c == '\\' && quote == '"') ++i;
            else if (c == quote) quote = 0;
        } else if (c == '"' || c == '\'') {
            quote = c;
        } else if (c == '#') {
            return line.substr(0, i);
        }
    }
    return line;
}

bool parse_value(std::string_view v, Value& out, std::string& err) {
    if (v.empty()) return err = "missing value", false;
    if (v.front() == '"' || v.front() == '\'') {
        const char q = v.front();
        std::string s;
        size_t i = 1;
        for (; i < v.size() && v[i] != q; ++i) {
            if (q == '"' && v[i] == '\\' && i + 1 < v.size()) {
                const char e = v[++i];
                s += e == 'n' ? '\n' : e == 't' ? '\t' : e;
            } else {
                s += v[i];
            }
        }
        if (i >= v.size()) return err = "unterminated string", false;
        if (!trim(v.substr(i + 1)).empty()) return err = "unexpected text after string", false;
        out.type = Value::String;
        out.s = std::move(s);
        return true;
    }
    if (v == "true" || v == "false") {
        out.type = Value::Bool;
        out.b = v == "true";
        return true;
    }
    std::string num;
    for (char c : v)
        if (c != '_') num += c;
    if (num.empty()) return err = "can't parse value '" + std::string(v) + "'", false;
    const char* first = num.data();
    const char* last = num.data() + num.size();
    const bool neg = !num.empty() && num[0] == '-';
    const size_t sign = (!num.empty() && (num[0] == '-' || num[0] == '+')) ? 1 : 0;
    if (num.size() > sign + 2 && num[sign] == '0' && (num[sign + 1] == 'x' || num[sign + 1] == 'X')) {
        uint64_t u = 0;
        const auto r = std::from_chars(first + sign + 2, last, u, 16);
        if (r.ec != std::errc() || r.ptr != last) return err = "bad hex number '" + std::string(v) + "'", false;
        out.type = Value::Int;
        out.i = neg ? -int64_t(u) : int64_t(u);
        return true;
    }
    int64_t i = 0;
    auto r = std::from_chars(first + (num[0] == '+' ? 1 : 0), last, i);
    if (r.ec == std::errc() && r.ptr == last) {
        out.type = Value::Int;
        out.i = i;
        return true;
    }
    double f = 0;
    r = std::from_chars(first + (num[0] == '+' ? 1 : 0), last, f);
    if (r.ec == std::errc() && r.ptr == last) {
        out.type = Value::Float;
        out.f = f;
        return true;
    }
    return err = "can't parse value '" + std::string(v) + "' (strings need quotes)", false;
}

bool parse_toml(std::string_view text, std::map<std::string, Table>& out, std::string& error) {
    std::string section;
    int line_no = 0;
    while (!text.empty()) {
        const size_t nl = text.find('\n');
        const std::string_view raw = text.substr(0, nl);
        text = nl == std::string_view::npos ? std::string_view{} : text.substr(nl + 1);
        ++line_no;
        const std::string_view line = trim(strip_comment(raw));
        if (line.empty()) continue;
        const std::string where = "line " + std::to_string(line_no) + ": ";
        if (line.front() == '[') {
            if (line.size() < 3 || line.back() != ']' || line[1] == '[')
                return error = where + "bad section header", false;
            section = std::string(trim(line.substr(1, line.size() - 2)));
            out[section];
            continue;
        }
        const size_t eq = line.find('=');
        if (eq == std::string_view::npos) return error = where + "expected key = value", false;
        const std::string key(trim(line.substr(0, eq)));
        if (key.empty() || !std::all_of(key.begin(), key.end(), [](char c) {
                return std::isalnum(static_cast<unsigned char>(c)) || c == '_' || c == '-';
            }))
            return error = where + "bad key '" + key + "'", false;
        Value v;
        v.line = line_no;
        std::string err;
        if (!parse_value(trim(line.substr(eq + 1)), v, err)) return error = where + err, false;
        if (!out[section].emplace(key, std::move(v)).second) return error = where + "duplicate key '" + key + "'", false;
    }
    return true;
}

// ---- Profile fields -------------------------------------------------------------------------

struct Reader {
    const Table& t;
    const char* section;
    std::string& error;

    const Value* find(const char* key, bool required) {
        const auto it = t.find(key);
        if (it != t.end()) return &it->second;
        if (required) error = std::string("[") + section + "] needs '" + key + "'";
        return nullptr;
    }
    std::string where(const Value& v) { return "line " + std::to_string(v.line) + ": "; }

    bool uint(const char* key, uint32_t& out, bool required, uint32_t max) {
        const Value* v = find(key, required);
        if (v == nullptr) return !required;
        if (v->type != Value::Int || v->i < 0 || v->i > int64_t(max))
            return error = where(*v) + key + " must be an integer in 0.." + std::to_string(max), false;
        out = uint32_t(v->i);
        return true;
    }
    // One of `names`; returns the index.
    bool choice(const char* key, std::initializer_list<const char*> names, int& out, bool required) {
        const Value* v = find(key, required);
        if (v == nullptr) return !required;
        int i = 0;
        if (v->type == Value::String)
            for (const char* n : names) {
                if (v->s == n) return out = i, true;
                ++i;
            }
        std::string list;
        for (const char* n : names) list += std::string(list.empty() ? "" : " | ") + n;
        return error = where(*v) + key + " must be one of: " + list, false;
    }
};

constexpr shader_stage kStages[] = {shader_stage::vertex, shader_stage::pixel,   shader_stage::geometry,
                                    shader_stage::hull,   shader_stage::domain, shader_stage::compute};

bool read_camera(const Table& t, CameraProfile& c, std::string& error) {
    static const char* const kKeys[] = {"stage",  "slot",        "space",       "size",  "layout",
                                        "view_offset", "proj_offset", "major", "handed", "latch",
                                        "units_per_meter", "translation_offset", "translation", "up"};
    for (const auto& [key, v] : t)
        if (std::none_of(std::begin(kKeys), std::end(kKeys), [&](const char* k) { return key == k; }))
            return error = "line " + std::to_string(v.line) + ": unknown key '" + key + "' in [camera]", false;

    Reader r{t, "camera", error};
    int stage = 0, layout = 0, major = 0, handed = 0, latch = 0, up = 0;
    if (!r.choice("stage", {"vertex", "pixel", "geometry", "hull", "domain", "compute"}, stage, true) ||
        !r.uint("slot", c.key.slot, true, 255) || !r.uint("space", c.key.space, false, 0xFFFFFFEFu) ||
        !r.uint("size", c.key.size, false, 1u << 20) ||
        !r.choice("layout",
                  {"view+proj", "viewproj+proj", "invview+proj", "invviewproj+proj", "viewproj", "invviewproj",
                   "modelview"},
                  layout, true) ||
        !r.uint("view_offset", c.view_offset, true, 1u << 20))
        return false;
    c.layout = CameraLayout(layout);
    if (c.single_matrix()) {
        if (const Value* v = r.find("proj_offset", false))
            return error = r.where(*v) + "proj_offset doesn't apply to layout " + layout_name(c.layout), false;
    } else if (!r.uint("proj_offset", c.proj_offset, true, 1u << 20)) {
        return false;
    }
    if (!r.choice("major", {"row", "column"}, major, false) || !r.choice("handed", {"left", "right"}, handed, false) ||
        !r.choice("latch", {"first", "last", "common"}, latch, false) || !r.choice("up", {"y", "z"}, up, false))
        return false;
    c.key.stage = kStages[stage];
    c.column_major = major == 1;
    c.right_handed = handed == 1;
    c.latch = Latch(latch);
    c.z_up = up == 1;
    if (const Value* v = r.find("units_per_meter", false)) {
        const double u = v->type == Value::Float ? v->f : v->type == Value::Int ? double(v->i) : 0;
        if (!(u > 0 && u < 1e6)) return error = r.where(*v) + "units_per_meter must be a positive number", false;
        c.units_per_meter = float(u);
    }
    if (const Value* v = r.find("translation_offset", false)) {
        if (c.model_view())
            return error = r.where(*v) + "translation_offset doesn't apply to layout " + layout_name(c.layout), false;
        int sign = 0;
        if (!r.uint("translation_offset", c.translation_offset, true, 1u << 20) ||
            !r.choice("translation", {"add", "subtract"}, sign, true))
            return false;
        c.has_translation = true;
        c.translation_subtract = sign == 1;
    } else if (const Value* sign = r.find("translation", false)) {
        return error = r.where(*sign) + "translation needs translation_offset", false;
    }
    if (c.view_offset % 4 != 0 || c.proj_offset % 4 != 0 || c.translation_offset % 4 != 0)
        return error = "offsets must be multiples of 4 bytes", false;
    if (!c.single_matrix()) {
        const uint32_t lo = std::min(c.view_offset, c.proj_offset), hi = std::max(c.view_offset, c.proj_offset);
        if (hi - lo < 64) return error = "view_offset and proj_offset overlap", false;
    }
    if (c.key.size != 0 && c.window_offset() + c.window_size() > c.key.size)
        return error = "matrices extend past the buffer size", false;
    return true;
}

// ---- Matrix math (double, row-major, row-vector: matrix.h) ----------------------------------

using mat::finite;
using mat::inverse;
using mat::load;
using mat::Mat;
using mat::mul;
using mat::store;

// A rigid world->view transform: last column (0,0,0,1), rotation part with |det| ~ 1.
bool plausible_view(const float v[16]) {
    if (std::abs(v[3]) > 1e-3f || std::abs(v[7]) > 1e-3f || std::abs(v[11]) > 1e-3f || std::abs(v[15] - 1) > 1e-3f)
        return false;
    const double det = double(v[0]) * (double(v[5]) * v[10] - double(v[6]) * v[9]) -
                       double(v[1]) * (double(v[4]) * v[10] - double(v[6]) * v[8]) +
                       double(v[2]) * (double(v[4]) * v[9] - double(v[5]) * v[8]);
    return std::abs(std::abs(det) - 1.0) < 1e-2;
}

bool fail(std::string* why, const char* msg) {
    if (why) *why = msg;
    return false;
}

}  // namespace

uint32_t CameraProfile::window_offset() const {
    uint32_t lo = single_matrix() ? view_offset : std::min(view_offset, proj_offset);
    if (has_translation) lo = std::min(lo, translation_offset);
    return lo;
}
uint32_t CameraProfile::window_size() const {
    uint32_t hi = single_matrix() ? view_offset + 64 : std::max(view_offset, proj_offset) + 64;
    if (has_translation) hi = std::max(hi, translation_offset + 12);
    return hi - window_offset();
}

bool parse_profile(std::string_view text, Profile& out, std::string& error) {
    std::map<std::string, Table> doc;
    if (!parse_toml(text, doc, error)) return false;
    Profile p;
    const auto cam = doc.find("camera");
    if (cam == doc.end()) return error = "no [camera] section", false;
    if (!read_camera(cam->second, p.camera, error)) return false;
    p.has_camera = true;
    out = p;
    return true;
}

bool load_profile(const std::filesystem::path& path, Profile& out, std::string& error) {
    std::ifstream f(path, std::ios::binary);
    if (!f) return error = "can't open " + path.string(), false;
    std::stringstream ss;
    ss << f.rdbuf();
    return parse_profile(ss.str(), out, error);
}

std::string format_profile(const CameraProfile& c, std::string_view comment) {
    std::string s;
    while (!comment.empty()) {
        const size_t nl = comment.find('\n');
        s += "# " + std::string(comment.substr(0, nl)) + "\n";
        comment = nl == std::string_view::npos ? std::string_view{} : comment.substr(nl + 1);
    }
    if (!s.empty()) s += "\n";
    auto line = [&](const char* key, const std::string& value) {
        s += key;
        s.append(12 > std::strlen(key) ? 12 - std::strlen(key) : 1, ' ');
        s += "= " + value + "\n";
    };
    auto quoted = [](const char* v) { return std::string("\"") + v + "\""; };
    s += "[camera]\n";
    line("stage", quoted(stage_name(c.key.stage)));
    line("slot", std::to_string(c.key.slot));
    if (c.key.space != 0) line("space", std::to_string(c.key.space));
    if (c.key.size != 0) line("size", std::to_string(c.key.size));
    line("layout", quoted(layout_name(c.layout)));
    line("view_offset", std::to_string(c.view_offset));
    if (!c.single_matrix()) line("proj_offset", std::to_string(c.proj_offset));
    line("major", quoted(c.column_major ? "column" : "row"));
    line("handed", quoted(c.right_handed ? "right" : "left"));
    line("latch", quoted(latch_name(c.latch)));
    if (c.z_up) line("up", quoted("z"));
    if (c.has_translation) {
        line("translation_offset", std::to_string(c.translation_offset));
        line("translation", quoted(c.translation_subtract ? "subtract" : "add"));
    }
    if (c.units_per_meter != 1) {
        char buf[32];
        std::snprintf(buf, sizeof(buf), "%.9g", double(c.units_per_meter));
        line("units_per_meter", buf);
    }
    return s;
}

const char* layout_name(CameraLayout layout) {
    switch (layout) {
        case CameraLayout::ViewAndProj: return "view+proj";
        case CameraLayout::ViewProjAndProj: return "viewproj+proj";
        case CameraLayout::InvViewAndProj: return "invview+proj";
        case CameraLayout::InvViewProjAndProj: return "invviewproj+proj";
        case CameraLayout::ViewProj: return "viewproj";
        case CameraLayout::InvViewProj: return "invviewproj";
        case CameraLayout::ModelView: return "modelview";
    }
    return "?";
}

const char* latch_name(Latch latch) {
    switch (latch) {
        case Latch::First: return "first";
        case Latch::Last: return "last";
        case Latch::Common: return "common";
    }
    return "?";
}

const char* stage_name(shader_stage stage) {
    switch (stage) {
        case shader_stage::vertex: return "vertex";
        case shader_stage::pixel: return "pixel";
        case shader_stage::geometry: return "geometry";
        case shader_stage::hull: return "hull";
        case shader_stage::domain: return "domain";
        case shader_stage::compute: return "compute";
        default: return "?";
    }
}

bool decode_camera(const CameraProfile& c, const uint8_t* window, size_t window_size, float view[16], float proj[16],
                   std::string* why) {
    if (c.model_view()) return fail(why, "a model-view camera needs the solver");
    if (window_size < c.window_size()) return fail(why, "latched window too small");
    const uint8_t* a = window + (c.view_offset - c.window_offset());
    const Mat first = load(a, c.column_major);
    if (!finite(first)) return fail(why, "non-finite values");

    // Camera-relative matrices: world -> that space first (row vectors: p * T = p ± t).
    Mat to_relative = mat::identity();
    if (c.has_translation) {
        float t[3];
        std::memcpy(t, window + (c.translation_offset - c.window_offset()), sizeof(t));
        for (int i = 0; i < 3; ++i) {
            if (!std::isfinite(t[i])) return fail(why, "non-finite translation");
            to_relative.m[3][i] = c.translation_subtract ? -double(t[i]) : double(t[i]);
        }
    }

    Mat v, inv;
    if (c.single_matrix()) {
        Mat vp = first, p;
        if (c.layout == CameraLayout::InvViewProj && !inverse(first, vp))
            return fail(why, "inverse view-projection is singular");
        if (!decompose_view_proj(vp, v, p)) return fail(why, "not a perspective view-projection");
        if (c.has_translation) v = mul(to_relative, v);
        store(v, view);
        store(p, proj);
        if (!analyze_projection(proj).valid) return fail(why, "proj is not a perspective projection");
        return true;
    }

    const uint8_t* b = window + (c.proj_offset - c.window_offset());
    const Mat p = load(b, c.column_major);
    if (!finite(p)) return fail(why, "non-finite values");
    store(p, proj);  // float -> double -> float is exact

    switch (c.layout) {
        case CameraLayout::ViewAndProj:
            v = first;  // exact, like proj
            break;
        case CameraLayout::ViewProjAndProj:
            if (!inverse(p, inv)) return fail(why, "projection is singular");
            v = mul(first, inv);
            break;
        case CameraLayout::InvViewAndProj:
            if (!inverse(first, v)) return fail(why, "inverse view is singular");
            break;
        case CameraLayout::InvViewProjAndProj: {
            Mat vp;
            if (!inverse(first, vp)) return fail(why, "inverse view-projection is singular");
            if (!inverse(p, inv)) return fail(why, "projection is singular");
            v = mul(vp, inv);
            break;
        }
        case CameraLayout::ViewProj:
        case CameraLayout::InvViewProj:
        case CameraLayout::ModelView: break;  // handled above
    }
    if (c.has_translation) v = mul(to_relative, v);
    store(v, view);
    if (!analyze_projection(proj).valid) return fail(why, "proj is not a perspective projection");
    if (!plausible_view(view)) return fail(why, "view is not a rigid transform");
    return true;
}

bool decode_projection(const CameraProfile& c, const uint8_t* window, size_t window_size, float proj[16],
                       std::string* why) {
    if (window_size < 64) return fail(why, "latched window too small");
    const Mat p = load(window, c.column_major);
    if (!finite(p)) return fail(why, "non-finite values");
    store(p, proj);
    if (!analyze_projection(proj).valid) return fail(why, "proj is not a perspective projection");
    return true;
}

ProjectionInfo analyze_projection(const float p[16]) {
    ProjectionInfo info;
    // Row-vector: w_clip = z_view * p[11], z_clip = z_view * p[10] + p[14].
    const float s = p[11];
    if (std::abs(std::abs(s) - 1.0f) > 1e-3f || std::abs(p[15]) > 1e-3f || p[0] == 0 || p[5] == 0) return info;
    info.right_handed = s < 0;
    // With d = distance in front of the camera, ndc_z = a + b / d.
    const double a = double(p[10]) * s, b = p[14];
    const double inf = std::numeric_limits<double>::infinity();
    double d0 = a == 0 ? inf : -b / a;        // where depth = 0
    double d1 = a == 1 ? inf : b / (1 - a);   // where depth = 1
    // Depth tends to `a` far away. Strictly inside (0, 1), the far end is never reached: an infinite
    // projection that keeps a little headroom (UE3: 1 - 1e-3 at infinity).
    if (a > 0 && a < 1) {
        if (!(d0 > 0) && d1 > 0) d0 = inf;
        if (!(d1 > 0) && d0 > 0) d1 = inf;
    }
    // Near and far close together: every depth value means about the same distance, so it can't place
    // anything (and trivially "matches" any depth image). Scene cameras span orders of magnitude.
    if (!(d0 > 0) || !(d1 > 0) || std::max(d0, d1) < 10 * std::min(d0, d1)) return info;
    info.reversed = d1 < d0;
    info.near_z = float(std::min(d0, d1));
    info.far_z = float(std::max(d0, d1));
    info.fov_y_deg = float(2.0 * std::atan(1.0 / std::abs(double(p[5]))) * 180.0 / 3.14159265358979323846);
    info.aspect = std::abs(p[5] / p[0]);
    info.valid = true;
    return info;
}

void normalize_pose(float view[16], float proj[16], bool right_handed, float units_per_meter, bool z_up) {
    // View-space handedness, with F = diag(1, 1, -1, 1): view' = view * F, proj' = F * proj. The same
    // clip space; view space comes out left-handed, the world is left as it is.
    if (right_handed) {
        for (int i = 0; i < 4; ++i) {
            view[i * 4 + 2] = -view[i * 4 + 2];  // column 2
            proj[2 * 4 + i] = -proj[2 * 4 + i];
        }
    }
    // The world, with a basis change B (p' = p * B, so view' = B^T * view: its rows mixed). z-up to
    // y-up: (x, y, z) -> (x, z, -y), a rotation. Then, if the world is mirrored against view space
    // (det < 0, e.g. any right-handed world by now), z negated: up stays up. Points come out as p * B.
    auto row = [&](int i) { return view + i * 4; };
    if (z_up) {
        float y[4];
        std::memcpy(y, row(1), sizeof(y));
        std::memcpy(row(1), row(2), sizeof(y));
        for (int i = 0; i < 4; ++i) row(2)[i] = -y[i];
    }
    const double det = double(view[0]) * (double(view[5]) * view[10] - double(view[6]) * view[9]) -
                       double(view[1]) * (double(view[4]) * view[10] - double(view[6]) * view[8]) +
                       double(view[2]) * (double(view[4]) * view[9] - double(view[5]) * view[8]);
    if (det < 0)
        for (int i = 0; i < 4; ++i) row(2)[i] = -row(2)[i];
    // Units, with S = diag(k, k, k, 1), k = 1 / units_per_meter: view' = S^-1 * view * S scales the
    // translation only; proj' = S^-1 * proj * k (the same projection: clip space is homogeneous)
    // scales its last row only, so w stays z (analyze_projection's near/far come out in meters).
    if (units_per_meter != 1) {
        const float k = 1 / units_per_meter;
        for (int i = 0; i < 3; ++i) view[12 + i] *= k;
        for (int i = 12; i < 16; ++i) proj[i] *= k;
    }
}

}  // namespace lidar
