import {Vec2, Mat2} from './utils'
import {Polygon, Collision} from './shape';

const EPS = 0.0001

export class RigidBody {
  static idPool = 0;

  id: number;
  shape: Polygon;
  mass: number;
  inertia: number;
  frozen = false;
  pos = Vec2.ZERO;
  vel = Vec2.ZERO;
  acc = Vec2.ZERO;
  angle = 0;
  avel = 0;
  aacc = 0;
  restitution = 0.5;

  constructor (shape: Polygon, mass: number, inertia: number) {
    this.id = RigidBody.idPool++;
    this.shape = shape;
    this.mass = mass;
    this.inertia = inertia;
  }

  freeze() {
    this.frozen = true;
  }
  unfreeze() {
    this.frozen = false;
  }

  addForce(point: Vec2, force: Vec2) {
    if (this.frozen) {
      return;
    }
    const r = this.pos.to(point);
    const torque = r.cross(force);
    this.acc = this.acc.add(force.times(1 / this.mass));
    this.aacc += torque / this.inertia;
  }

  velAt(point: Vec2) {
    const r = this.pos.to(point);
    const rotVel = r.rot90().times(this.avel);
    return this.vel.add(rotVel);
  }

  movedShape() {
    const rotMat = Mat2.rotate(this.angle);
    const vl = this.shape.vertices.map(
      (v) => rotMat.mulvec(v).add(this.pos)
    );
    return new Polygon(vl);
  }

  /** ワールド座標のポリゴンから重心等を計算してインスタンスを生成する */
  static from_polygon(poly: Polygon, density = 1.0) {
    const v0 = poly.vertex(0);
    function forTriangles(f: (va:Vec2, vb:Vec2, subM:number) => void) {
      for (let i = 0; i < poly.vertices.length - 1; i++) {
        const va = poly.vertex(i).sub(v0);
        const vb = poly.vertex(i + 1).sub(v0);
        const subM = Math.abs(va.cross(vb)) / 2 * density;
        f(va, vb, subM);
      }
    }

    // 質量・重心を計算
    let mass = 0;
    let v0g = Vec2.ZERO;
    forTriangles((va, vb, subM) => {
      mass += subM;
      v0g = v0g.add(va.add(vb).times(subM/3));
    })
    v0g = v0g.times(1/mass)

    // 慣性モーメントを計算
    let inertia = 0;
    forTriangles((va, vb, subM) => {
      const subI = subM/18 * (va.normSq() +vb.normSq() -va.dot(vb));
      const d2 = va.add(vb).times(1/3).to(v0g).normSq();
      inertia += subI +subM*d2;
    });

    // ポリゴンを生成
    const center = v0.add(v0g);
    const vs = poly.vertices.map((v) => center.to(v))
    const shape = new Polygon(vs);

    // おわり
    const body = new RigidBody(shape, mass, inertia);
    body.pos = center;
    return body;
  }
}

export class World {
  gravity = Vec2.c(0, 9.8);
  bodies: RigidBody[] = [];

  addBody(body: RigidBody) {
    this.bodies.push(body);
  }

  step(dt: number) {
    // ペナルティ
    const shapes = this.bodies.map((b) => b.movedShape());
    for (let i = 0; i < this.bodies.length; i++) {
      const body1 = this.bodies[i];
      const shape1 = shapes[i];
      for (let j = i + 1; j < this.bodies.length; j++) {
        const body2 = this.bodies[j];
        const shape2 = shapes[j];
        if (body1.frozen && body2.frozen)
          continue;
        const col = Collision.polygon_polygon(shape1, shape2);
        if (col === undefined)
          continue;

        for (const [p1, p2] of col.points) {
          const v12 = body2.velAt(p2).sub(body1.velAt(p1));
          const perpV12 = col.normal.dot(v12);

          const k = 50.0; // ばね定数
          const d = 0.5;  // ダンパ
          const force = col.normal.times(k*col.depth -d*perpV12);

          body1.addForce(p1, force.reverse());
          body2.addForce(p2, force);
        }
      }
    }

    // 重力
    for (const b of this.bodies) {
      b.addForce(b.pos, this.gravity.times(b.mass));
    }

    // 積分
    for (const b of this.bodies) {
      b.pos = b.pos.add(b.vel.times(dt));
      b.vel = b.vel.add(b.acc.times(dt));
      b.acc = Vec2.ZERO;
      b.angle += b.avel * dt;
      b.avel += b.aacc * dt;
      b.aacc = 0;
    }
  }
}