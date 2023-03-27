import {Vec2, Mat2} from './utils'
import {Polygon, Collision} from './shape';

export class RigidBody {
  static idPool = 0;

  id: number;
  shape: Polygon;
  mass: number;
  invMass: number;
  inertia: number;
  invInertia: number;
  frozen = false;
  pos = Vec2.ZERO;
  vel = Vec2.ZERO;
  acc = Vec2.ZERO;
  angle = 0;
  avel = 0;
  aacc = 0;
  restitution = 0.8;
  friction = 0.3;

  constructor (shape: Polygon, mass: number, inertia: number) {
    this.id = RigidBody.idPool++;
    this.shape = shape;
    this.mass = mass;
    this.invMass = 1 / mass;
    this.inertia = inertia;
    this.invInertia = 1 / inertia;
  }

  freeze() {
    this.frozen = true;
    this.invMass = 0;
    this.invInertia = 0;
  }
  unfreeze() {
    this.frozen = false;
    this.invMass = 1 / this.mass;
    this.invInertia = 1 / this.invInertia;
  }

  addForceLocal(r: Vec2, force: Vec2) {
    if (this.frozen) {
      return;
    }
    this.acc = this.acc.add(force.times(this.invMass));
    this.aacc += r.cross(force) * this.invInertia;
  }
  addForce(point: Vec2, force: Vec2) {
    const r = this.pos.to(point);
    this.addForceLocal(r, force);
  }
  addImpulseLocal(r: Vec2, impulse: Vec2) {
    if (this.frozen) {
      return;
    }
    this.vel = this.vel.add(impulse.times(this.invMass));
    this.avel += r.cross(impulse) * this.invInertia;
  }
  addImpulse(point: Vec2, impulse: Vec2) {
    const r = this.pos.to(point);
    this.addImpulseLocal(r, impulse);
  }

  velAtLocal(r: Vec2) {
    const rotVel = r.rot90().times(this.avel);
    return this.vel.add(rotVel);
  }
  velAt(point: Vec2) {
    const r = this.pos.to(point);
    return this.velAtLocal(r);
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

class Contact {
  body1: RigidBody;
  body2: RigidBody;
  normal: Vec2;
  r1: Vec2;
  r2: Vec2;
  impulse: Vec2;
  goalVel : number;

  invMassN: number;
  invMassT: number;
  invMassNT: number;

  constructor(dt: number, gravity: number, body1: RigidBody, body2: RigidBody, collision: Collision, pointi: number) {
    this.body1 = body1;
    this.body2 = body2;
    const normal = this.normal = collision.normal;
    const tangent = normal.rot90();
    const [point1, point2] = collision.points[pointi];
    const r1 = this.r1 = body1.pos.to(point1);
    const r2 = this.r2 = body2.pos.to(point2);
    this.impulse = Vec2.ZERO;
    // 目標となる法線方向の相対速度の算出
    const vel12 = body2.velAtLocal(r2).sub(body1.velAtLocal(r1)).dot(normal);
    let restitution = body1.restitution * body2.restitution;
    // 相対速度が十分小さいとき反発係数をゼロにする
    if (Math.abs(vel12) < gravity * dt * restitution * 5) {
      restitution = 0;
    }
    const velReaction = -restitution * vel12;
    const slop = gravity * dt * dt * 3;
    const velError = Math.min(collision.depth - slop, slop) / dt;
    this.goalVel = Math.max(velReaction, velError, 0);
    // 事前計算
    this.invMassN = body1.invMass + body2.invMass
      + body1.invInertia * r1.cross(normal)**2
      + body2.invInertia * r2.cross(normal)**2;
    this.invMassT = body1.invMass + body2.invMass
      + body1.invInertia * r1.cross(tangent)**2
      + body2.invInertia * r2.cross(tangent)**2;
    this.invMassNT =
        body1.invInertia * r1.cross(normal) * r1.cross(tangent)
      + body2.invInertia * r2.cross(normal) * r2.cross(tangent);
  }

  solve() {
    const r1 = this.r1;
    const r2 = this.r2;
    const normal = this.normal;
    const tangent = normal.rot90();
    const invMassN = this.invMassN;
    const invMassT = this.invMassT;
    const invMassNT = this.invMassNT;

    // 前回与えた撃力を打ち消す
    this.body1.addImpulseLocal(r1, this.impulse);
    this.body2.addImpulseLocal(r2, this.impulse.reverse());

    const vel12 = this.body2.velAtLocal(r2).sub(this.body1.velAtLocal(r1));
    const velDiff = this.goalVel - normal.dot(vel12);

    if (velDiff <= 0) {
      // 目標の速度を達成している場合は何もしない
      return;
    }

    // 摩擦
    let impulseT = -(invMassN*tangent.dot(vel12) +invMassNT*velDiff) / (invMassT*invMassN -invMassNT**2);
    // 垂直抗力
    let impulseN = (velDiff -invMassNT*impulseT) / invMassN;

    // 摩擦係数
    const friction = this.body1.friction * this.body2.friction;

    // 静止が可能かどうか判断
    if (Math.abs(impulseT) > friction * impulseN) {
      // 動摩擦
      const sign = Math.sign(impulseT);
      const invMass = invMassN +sign*friction*invMassNT;
      if (invMass <= 0) {
        // 衝突なし (この分岐、本当に踏むことあるのか？)
        console.log(sign, invMassNT);
        return;
      }
      impulseN = velDiff / invMass;
      impulseT = sign * friction * impulseN;
    }

    this.impulse = tangent.times(impulseT).add(normal.times(impulseN));
    this.body1.addImpulseLocal(r1, this.impulse.reverse());
    this.body2.addImpulseLocal(r2, this.impulse);
  }
}

export class World {
  gravity = Vec2.c(0, 9.8);
  bodies: RigidBody[] = [];

  addBody(body: RigidBody) {
    this.bodies.push(body);
  }

  step(dt: number) {
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

    // 衝突検出
    const gNorm = Math.sqrt(this.gravity.normSq())
    const constraints: Contact[] = [];
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

        for (let i = 0; i < col.points.length; i++) {
          constraints.push(
            new Contact(dt, gNorm, body1, body2, col, i)
          );
        }
      }
    }

    // 衝突応答
    const loop = 50;
    for (let i = 0; i < loop; i++) {
      for (const cnst of constraints) {
        cnst.solve();
      }
    }
  }
}