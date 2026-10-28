shader_type spatial;
render_mode cull_disabled;

// Linear grey albedo. A generated, mipmapped noise texture supplies aggregate.
uniform vec3 asphalt_color = vec3(0.14, 0.15, 0.16);
uniform sampler2D aggregate : filter_linear_mipmap_anisotropic, repeat_enable;
varying vec2 ground_position;

void vertex() {
 ground_position = (MODEL_MATRIX * vec4(VERTEX, 1.0)).xz;
}

void fragment() {
 // Mipmaps average fine grain in distant pixels, avoiding camera shimmer.
 float grain = texture(aggregate, ground_position / 1.0).r - 0.5;
 float weathering = texture(aggregate, ground_position / 32.0).r - 0.5;
 ALBEDO = asphalt_color * (1.0 + weathering * 0.12 + grain * 0.55);
 ROUGHNESS = 0.95;
 SPECULAR = 0.12;
}
