shader_type spatial;
render_mode depth_draw_opaque;

// Uniforms let you tweak from the Inspector
// e.g., wave amplitude and speed.
uniform float time_scale : hint_range(0.0, 10.0) = 2.0;
uniform float side_to_side : hint_range(0.0, 10.0) = 0.0;
uniform float pivot : hint_range(-10.0, 10.0) = 0.0;
uniform float wave : hint_range(0.0, 10.0) = 1.0;
uniform float twist : hint_range(0.0, 10.0) = 1.0;
uniform vec4 fish_color : source_color = vec4(1.0, 0.5, 0.0, 1.0);

// If you want an additional frequency factor, you could add:
// uniform float wave_frequency = 5.0;

void vertex() {
	float time = TIME * time_scale;
	VERTEX.x += cos(time) * side_to_side;
	//angle is scaled by 0.1 so that the fish only pivots and doesn't rotate all the way around
	float pivot_angle = cos(time) * 0.1 * pivot;
	mat2 rotation_matrix = mat2(vec2(cos(pivot_angle), -sin(pivot_angle)), vec2(sin(pivot_angle), cos(pivot_angle)));
	VERTEX.xz = rotation_matrix * VERTEX.xz;
	float body = (VERTEX.z + 1.0) / 2.0; //for a fish centered at (0, 0) with a length of 2
	//wave is a uniform float
	VERTEX.x += cos(time + body) * wave;

	//twist is a uniform float
	float twist_angle = cos(time + body) * 0.3 * twist;
	mat2 twist_matrix = mat2(vec2(cos(twist_angle), -sin(twist_angle)), vec2(sin(twist_angle), cos(twist_angle)));
	VERTEX.xy = twist_matrix * VERTEX.xy;
}

void fragment(){
	ALBEDO = fish_color.rgb;
}
