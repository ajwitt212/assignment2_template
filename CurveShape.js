import { tiny, defs } from './examples/common.js'

// Pull these names into this module's scope for convenience:
const { vec3, vec4, color, Mat4, Shape, Material, Shader, Texture, Component } = tiny;

class CurveShape extends Shape {
    constructor(curve_function, sample_cnt, curve_color = color(1, 0, 0, 1)) {
        // curve function: (t) => vec3
        super("position", "normal")

        this.material = { shader: new defs.Phong_Shader(), ambient: 1.0, color: curve_color };
        this.sample_count = sample_cnt;

        if (curve_function && this.sample_count) {
            for (let i = 0; i < this.sample_count; i++) {
                let t = i / this.sample_count;
                this.arrays.position.push(curve_function(t));
                this.arrays.normal.push(vec3(0, 0, 0)); // normal required for phong
            }
        }
    }

    draw(webgl_manager, uniforms) {
        // call super with "LINE_STRIP" to draw the curve as a line
        let no_transform = Mat4.identity();    // no transformation
        super.draw(webgl_manager, uniforms, no_transform, this.material, "LINE_STRIP");
    }

    update(webgl_manager, uniforms, curve_function) {
        // update curve by updating positions array
        if (curve_function && this.sample_count) {
            for (let i = 0; i < this.sample_count + 1; i++) {
                let t = 1.0 * i / this.sample_count;
                this.arrays.position[i] = curve_function(t);
            }
        }
        // this.arrays.position.forEach((v, i) => v = curve_function(i / this.sample_count));
        this.copy_onto_graphics_card(webgl_manager.context);
        // Note: vertex count is not changing, so no need to call update_vertex_count
        // Not tested if possible to change vertex count
    }
};

export default CurveShape;