import { tiny, defs } from './examples/common.js';

// Pull these names into this module's scope for convenience:
const { vec3, vec4, color, Mat4, Shape, Material, Shader, Texture, Component } = tiny;

const shapes = {
    'sphere': new defs.Subdivision_Sphere(5),
};

export
    const Articulated_Human =
        class Articulated_Human {
            constructor() {
                const sphere_shape = shapes.sphere;

                // torso node
                const torso_transform = Mat4.scale(1, 2.5, 0.5);
                this.torso_node = new Node("torso", sphere_shape, torso_transform);
                // root->torso
                const root_location = Mat4.translation(0, 5, 0);
                this.root = new Arc("root", null, this.torso_node, root_location);

                // head node
                let head_transform = Mat4.scale(.6, .6, .6);
                head_transform.pre_multiply(Mat4.translation(0, .6, 0));
                this.head_node = new Node("head", sphere_shape, head_transform);
                // torso->neck->head
                const neck_location = Mat4.translation(0, 2.5, 0);
                this.neck = new Arc("neck", this.torso_node, this.head_node, neck_location);
                this.torso_node.children_arcs.push(this.neck);

                // right upper arm node
                let ru_arm_transform = Mat4.scale(1.2, .2, .2);
                ru_arm_transform.pre_multiply(Mat4.translation(1.2, 0, 0));
                this.ru_arm_node = new Node("ru_arm", sphere_shape, ru_arm_transform);
                // torso->r_shoulder->ru_arm
                const r_shoulder_location = Mat4.translation(0.6, 2, 0);
                this.r_shoulder = new Arc("r_shoulder", this.torso_node, this.ru_arm_node, r_shoulder_location);
                this.torso_node.children_arcs.push(this.r_shoulder)
                this.r_shoulder.set_dof(true, true, true);

                // right lower arm node
                let rl_arm_transform = Mat4.scale(1, .2, .2);
                rl_arm_transform.pre_multiply(Mat4.translation(1, 0, 0));
                this.rl_arm_node = new Node("rl_arm", sphere_shape, rl_arm_transform);
                // ru_arm->r_elbow->rl_arm
                const r_elbow_location = Mat4.translation(2.4, 0, 0);
                this.r_elbow = new Arc("r_elbow", this.ru_arm_node, this.rl_arm_node, r_elbow_location);
                this.ru_arm_node.children_arcs.push(this.r_elbow)
                this.r_elbow.set_dof(true, true, false);

                // right hand node
                let r_hand_transform = Mat4.scale(.4, .3, .2);
                r_hand_transform.pre_multiply(Mat4.translation(0.4, 0, 0));
                this.r_hand_node = new Node("r_hand", sphere_shape, r_hand_transform);
                // rl_arm->r_wrist->r_hand
                const r_wrist_location = Mat4.translation(2, 0, 0);
                this.r_wrist = new Arc("r_wrist", this.rl_arm_node, this.r_hand_node, r_wrist_location);
                this.rl_arm_node.children_arcs.push(this.r_wrist);
                this.r_wrist.set_dof(true, false, true);

                // add the only end-effector
                const r_hand_end_local_pos = vec4(0.8, 0, 0, 1);
                this.end_effector = new End_Effector("right_hand", this.r_wrist, r_hand_end_local_pos);
                this.r_wrist.end_effector = this.end_effector;

                // here I only use 7 dof
                this.dof = 7;
                this.Jacobian = null;
                this.theta = [0, 0, 0, 0, 0, 0, 0];
                this.apply_theta();
            }

            // mapping from global theta to each joint theta
            apply_theta() {
                this.r_shoulder.update_articulation(this.theta.slice(0, 3));
                this.r_elbow.update_articulation(this.theta.slice(3, 5));
                this.r_wrist.update_articulation(this.theta.slice(5, 7));
            }

            calculate_Jacobian() {
                let J = new Array(3);
                for (let i = 0; i < 3; i++) {
                    J[i] = new Array(this.dof).fill(0);
                }

                const delta_theta = 0.001; // Small perturbation
                const original_theta = this.theta.slice();
                const original_pos = this.get_end_effector_position();

                for (let i = 0; i < this.dof; i++) {
                    // Perturb theta
                    this.theta[i] += delta_theta;
                    this.apply_theta();

                    // Observe change in end effector position
                    const new_pos = this.get_end_effector_position();

                    // Revert theta
                    this.theta = original_theta.slice();
                    this.apply_theta();

                    // Calculate partial derivative for each dimension
                    for (let j = 0; j < 3; j++) { // x, y, z
                        J[j][i] = (new_pos[j] - original_pos[j]) / delta_theta;
                    }
                }

                return J;
            }

            iterative_ik(target_pos, iterations = 100) {
                for (let i = 0; i < iterations; i++) {
                    // if the end effector is close enough to the target, break
                    if (this.get_end_effector_position().minus(target_pos).norm() < 0.01) {
                        break;
                    }
                    // calc Jacobian
                    const J = this.calculate_Jacobian();
                    // calc dx
                    const dx_vec = target_pos.minus(this.get_end_effector_position());
                    const dx = [[dx_vec[0]], [dx_vec[1]], [dx_vec[2]]];
                    // calc dtheta
                    const dtheta = this.calculate_delta_theta2(J, dx);
                    // update and apply theta
                    this.theta = this.theta.map((v, i) => v + dtheta[i][0]);
                    this.apply_theta();
                }
            }

            calculate_delta_theta2(J, dx) {
                const J_square = math.multiply(math.transpose(J), J);
                for (let i = 0; i < J_square.length; i++) {
                    J_square[i][i] += 0.0001;
                }
                const J_plus = math.multiply(math.inv(J_square), math.transpose(J));
                const dtheta = math.multiply(J_plus, dx);
                return dtheta;
            }

            get_end_effector_position() {
                // in this example, we only have one end effector.
                this.matrix_stack = [];
                this._rec_update(this.root, Mat4.identity());
                const v = this.end_effector.global_position; // vec4
                return vec3(v[0], v[1], v[2]);
            }

            _rec_update(arc, matrix) {
                if (arc !== null) {
                    const L = arc.location_matrix;
                    const A = arc.articulation_matrix;
                    matrix.post_multiply(L.times(A));
                    this.matrix_stack.push(matrix.copy());

                    if (arc.end_effector !== null) {
                        arc.end_effector.global_position = matrix.times(arc.end_effector.local_position);
                    }

                    const node = arc.child_node;
                    const T = node.transform_matrix;
                    matrix.post_multiply(T);

                    matrix = this.matrix_stack.pop();
                    for (const next_arc of node.children_arcs) {
                        this.matrix_stack.push(matrix.copy());
                        this._rec_update(next_arc, matrix);
                        matrix = this.matrix_stack.pop();
                    }
                }
            }

            draw(webgl_manager, uniforms, material) {
                this.matrix_stack = [];
                this._rec_draw(this.root, Mat4.identity(), webgl_manager, uniforms, material);
            }

            _rec_draw(arc, matrix, webgl_manager, uniforms, material) {
                if (arc !== null) {
                    const L = arc.location_matrix;
                    const A = arc.articulation_matrix;
                    matrix.post_multiply(L.times(A));
                    this.matrix_stack.push(matrix.copy());

                    const node = arc.child_node;
                    const T = node.transform_matrix;
                    matrix.post_multiply(T);
                    node.shape.draw(webgl_manager, uniforms, matrix, material);

                    matrix = this.matrix_stack.pop();
                    for (const next_arc of node.children_arcs) {
                        this.matrix_stack.push(matrix.copy());
                        this._rec_draw(next_arc, matrix, webgl_manager, uniforms, material);
                        matrix = this.matrix_stack.pop();
                    }
                }
            }

            debug(arc = null, id = null) {
                // this.theta = this.theta.map(x => x + 0.01);
                // this.apply_theta();
                const J = this.calculate_Jacobian();
                let dx = [[0], [-0.02], [0]];
                if (id === 2)
                    dx = [[-0.02], [0], [0]];
                const dtheta = this.calculate_delta_theta(J, dx);

                // const direction = new Array(this.dof);
                // let norm = 0;
                // for (let i = 0; i < direction.length; i++) {
                //     direction[i] = dtheta[i][0];
                //     norm += direction[i] ** 2.0;
                // }
                // norm = norm ** 0.5;
                // console.log(direction);
                // console.log(norm);
                // this.theta = this.theta.map((v, i) => v + 0.01 * (direction[i] / norm));
                this.theta = this.theta.map((v, i) => v + dtheta[i][0]);
                this.apply_theta();

                // if (arc === null)
                //     arc = this.root;
                //
                // if (arc !== this.root) {
                //     arc.articulation_matrix = arc.articulation_matrix.times(Mat4.rotation(0.02, 0, 0, 1));
                // }
                //
                // const node = arc.child_node;
                // for (const next_arc of node.children_arcs) {
                //     this.debug(next_arc);
                // }
            }
        }

class Node {
    constructor(name, shape, transform) {
        this.name = name;
        this.shape = shape;
        this.transform_matrix = transform;
        this.children_arcs = [];
    }
}

class Arc {
    constructor(name, parent, child, location) {
        this.name = name;
        this.parent_node = parent;
        this.child_node = child;
        this.location_matrix = location;
        this.articulation_matrix = Mat4.identity();
        this.end_effector = null;
        this.dof = {
            Rx: false,
            Ry: false,
            Rz: false,
        }
    }

    // Here I only implement rotational DOF
    set_dof(x, y, z) {
        this.dof.Rx = x;
        this.dof.Ry = y;
        this.dof.Rz = z;
    }

    update_articulation(theta) {
        this.articulation_matrix = Mat4.identity();
        let index = 0;
        if (this.dof.Rx) {
            this.articulation_matrix.pre_multiply(Mat4.rotation(theta[index], 1, 0, 0));
            index += 1;
        }
        if (this.dof.Ry) {
            this.articulation_matrix.pre_multiply(Mat4.rotation(theta[index], 0, 1, 0));
            index += 1;
        }
        if (this.dof.Rz) {
            this.articulation_matrix.pre_multiply(Mat4.rotation(theta[index], 0, 0, 1));
        }
    }
}

class End_Effector {
    constructor(name, parent, local_position) {
        this.name = name;
        this.parent = parent;
        this.local_position = local_position;
        this.global_position = null;
    }
}