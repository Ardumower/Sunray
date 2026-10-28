shader_type spatial;
render_mode cull_disabled;
uniform sampler2D grass_albedo : source_color, filter_linear_mipmap_anisotropic, repeat_enable;
uniform float tile_metres = 1.0;
uniform vec4 tint : source_color = vec4(1.0);
varying vec2 ground_position;
void vertex() {
 ground_position = (MODEL_MATRIX * vec4(VERTEX, 1.0)).xz;
}
void fragment() {
 vec2 uv = ground_position / tile_metres;
 vec3 grass = texture(grass_albedo, uv).rgb;
 // Slow modulation reduces obvious one-metre repetition without shifting blades.
 float variation = 0.95 + 0.05 * sin(ground_position.x * 0.31 + sin(ground_position.y * 0.27));
 ALBEDO = grass * tint.rgb * variation;
 ROUGHNESS = 1.0;
 SPECULAR = 0.10;
}
