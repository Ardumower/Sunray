shader_type spatial;
render_mode cull_disabled;
uniform vec2 panel_size = vec2(3.0, 3.0);
uniform float mesh_size = 0.08;
uniform float cord_width = 0.004;
uniform vec4 net_color : source_color = vec4(0.12, 0.19, 0.14, 1.0);
void fragment() {
 vec2 metres = UV * panel_size;
 vec2 distance_to_cord = abs(fract(metres / mesh_size + 0.5) - 0.5) * mesh_size;
 float distance_to_grid = min(distance_to_cord.x, distance_to_cord.y);
 ALBEDO = net_color.rgb;
 ROUGHNESS = 1.0;
 ALPHA = distance_to_grid <= cord_width * 0.5 ? 1.0 : 0.0;
 ALPHA_SCISSOR_THRESHOLD = 0.5;
}
