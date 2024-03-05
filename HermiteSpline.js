import { tiny, defs } from './examples/common.js'

// Pull these names into this module's scope for convenience:
const { vec3, vec4, color, Mat4, Shape, Material, Shader, Texture, Component } = tiny;

class HermiteSpline {
    constructor() {
        this.points = [];
        this.tangents = [];
        this.size = 0; // number of points - standing for how many curves to put for spline
    }

    add_point(x, y, z, tx, ty, tz) {
        this.points.push(vec3(x, y, z));
        this.tangents.push(vec3(tx, ty, tz));
        this.size += 1;
    }

    set_tangent(index, x, y, z) {
        this.tangents[index] = vec3(x, y, z);
    }

    set_point(index, x, y, z) {
        this.points[index] = vec3(x, y, z);
    }

    h0(t) {
        return 1 - t;
    }
    h1(t) {
        return t;
    }

    get_position(t) {
        /*  
            get position along curve at time t in [0, 1] across all this.size points
            strategy: find the two points t is between and interpolate between them
        */
        if (this.size < 2) {
            return vec3(0, 0, 0); // Not enough points to define a curve
        }

        // Find the segment of the curve that t is within
        const segmentIndex = Math.floor((this.size - 1) * t);
        const segmentT = (this.size - 1) * t - segmentIndex;

        // Ensure index is within bounds
        const A = Math.max(0, Math.min(this.size - 2, segmentIndex));
        const B = Math.max(0, Math.min(this.size - 1, segmentIndex + 1));

        // Get the points and tangents for the current segment
        const p0 = this.points[A];
        const p1 = this.points[B];
        const t0 = this.tangents[A];
        const t1 = this.tangents[B];

        // Hermite spline basis functions
        const h00 = (2 * segmentT * segmentT * segmentT) - (3 * segmentT * segmentT) + 1;
        const h10 = (segmentT * segmentT * segmentT) - (2 * segmentT * segmentT) + segmentT;
        const h01 = (-2 * segmentT * segmentT * segmentT) + (3 * segmentT * segmentT);
        const h11 = (segmentT * segmentT * segmentT) - (segmentT * segmentT);

        // Calculate the interpolated position
        return p0.times(h00).plus(t0.times(h10)).plus(p1.times(h01)).plus(t1.times(h11));
    }

    get_arc_length() {
        let length = 0;

        let sample_cnt = 1000;

        // Iterate over all points and get the length of the curve
        let prev = this.get_position(0);
        for (let i = 1; i < sample_cnt + 1; i++) {
            const t = i / sample_cnt;
            const curr = this.get_position(t);
            length += curr.minus(prev).norm();
            prev = curr;
        }
        return length
    }

    export_spline() {
        /* export the spline in text format as follows:
        <n>
        <c_x1 c_y1 c_z1 t_x1 t_y1 t_z1> 
        <c_x2 c_y2 c_z2 t_x2 t_y2 t_z2>
        ....
        <c_xn c_yn c_zn t_xn t_yn t_zn>
        where the prefix c_ refers to control points and the prefix t_ to tangents and n is the number of control points defined in the text.
        */
        let export_string = `${this.size}\n`;
        for (let i = 0; i < this.size; i++) {
            const point = this.points[i];
            const tangent = this.tangents[i];
            export_string += `${point[0]} ${point[1]} ${point[2]} ${tangent[0]} ${tangent[1]} ${tangent[2]}\n`;
        }
        return export_string;
    }
}

export default HermiteSpline;