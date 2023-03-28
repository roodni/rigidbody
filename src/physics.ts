import {Vec2, Mat2} from './utils'
import {Polygon, Collision} from './shape';

export class RigidBody {
  static idPool = 0;

  id: number;
  shape: Polygon | undefined;
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
  restitution = 0.5;
  friction = 0.3;

  noCollide: Set<number>; // 衝突を無視する剛体一覧

  constructor(shape: Polygon | undefined, mass: number, inertia: number) {
    this.id = RigidBody.idPool++;
    this.shape = shape;
    this.mass = mass;
    this.invMass = 1 / mass;
    this.inertia = inertia;
    this.invInertia = 1 / inertia;
    this.noCollide = new Set();
  }

  freeze() {
    this.frozen = true;
    this.invMass = 0;
    this.invInertia = 0;
    this.vel = Vec2.ZERO;
    this.acc = Vec2.ZERO;
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
    if (!this.shape) {
      return undefined;
    }
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

  static ether() {
    const body = new RigidBody(undefined, 1.0, 1.0);
    body.freeze();
    return body;
  }
}


export type DistanceJoint = {
  body1: RigidBody;
  body2: RigidBody;
  r1: Vec2;
  r2: Vec2;
  max: number;
  noCollide: boolean;
};
export const DistanceJoint = {
  pin(body1: RigidBody, body2: RigidBody, point: Vec2, noCollide=true): DistanceJoint {
    const rot1 = Mat2.rotate(-body1.angle);
    const rot2 = Mat2.rotate(-body2.angle);
    return {
      body1, body2,
      r1: rot1.mulvec(body1.pos.to(point)),
      r2: rot2.mulvec(body2.pos.to(point)),
      max: 0,
      noCollide
    };
  }
};

interface Constraint {
  solve(): void;
}

export class Contact implements Constraint {
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

  isFrictionStatic: boolean;

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
    if (Math.abs(vel12) < gravity * dt * restitution * 10) {
      restitution = 0;
    }
    const velReaction = -restitution * vel12;
    const slop = gravity * dt * dt * 3;
    const velError = Math.min(collision.depth - slop, slop) / dt;
    this.goalVel = Math.max(velReaction, velError);
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

    this.isFrictionStatic = true;
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
    this.isFrictionStatic = true;
    if (Math.abs(impulseT) > friction * impulseN) {
      // 動摩擦
      this.isFrictionStatic = false;
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

class DistanceConstraint implements Constraint {
  joint: DistanceJoint;
  r1: Vec2;
  r2: Vec2;
  goalVel?: Vec2;

  invMassX: number;
  invMassY: number;
  invMassXY: number;

  constructor(dt: number, joint: DistanceJoint) {
    this.joint = joint;
    const b1 = joint.body1;
    const b2 = joint.body2;
    const rot1 = Mat2.rotate(b1.angle);
    const rot2 = Mat2.rotate(b2.angle);

    const p1 = rot1.mulvec(joint.r1).add(b1.pos);
    const p2 = rot2.mulvec(joint.r2).add(b2.pos);
    const p12 = p1.to(p2);
    const center = p1.add(p12.times(0.5));
    const r1 = this.r1 = b1.pos.to(center);
    const r2 = this.r2 = b2.pos.to(center);

    const d12 = p12.norm();
    if (d12 > joint.max) {
      const vel = -(d12 - joint.max) / dt;
      this.goalVel = p12.times(vel);
    }

    this.invMassX = b1.invMass +b2.invMass +b1.invInertia*r1.y**2 +b2.invInertia*r2.y**2;
    this.invMassY = b1.invMass +b2.invMass +b1.invInertia*r1.x**2 +b2.invInertia*r2.x**2;
    this.invMassXY = -b1.invInertia*r1.x*r1.y -b2.invInertia*r2.x*r2.y;
  }

  solve() {
    if (!this.goalVel) {
      return;
    }

    const vel1 = this.joint.body1.velAtLocal(this.r1);
    const vel2 = this.joint.body2.velAtLocal(this.r2);
    const vel12 = vel2.sub(vel1);
    const velDiff = this.goalVel.sub(vel12);

    const ix = Vec2.c(this.invMassY, -this.invMassXY).dot(velDiff);
    const iy = Vec2.c(-this.invMassXY, this.invMassX).dot(velDiff);
    const impulse = Vec2.c(ix, iy).times(1/(this.invMassX*this.invMassY -this.invMassXY**2));
    this.joint.body1.addImpulseLocal(this.r1, impulse.reverse());
    this.joint.body2.addImpulseLocal(this.r2, impulse);
  }
}


export class World {
  gravity = Vec2.c(0, 9.8);
  ether: RigidBody;
  bodies: RigidBody[] = [];
  joints: DistanceJoint[] = [];
  contacts: Contact[] = [];

  constructor() {
    this.ether = RigidBody.ether();
    this.addBody(this.ether);
  }

  addBody(body: RigidBody) {
    this.bodies.push(body);
  }

  addJoint(joint: DistanceJoint) {
    this.joints.push(joint);
    // 接続したオブジェクト同士の当たり判定を消す
    if (joint.noCollide) {
      // ジョイントの削除を可能にするには
      // noCollideの持ち方にもっと工夫が必要そう (参照カウントとか?)
      joint.body1.noCollide.add(joint.body2.id);
      joint.body2.noCollide.add(joint.body1.id);
    }
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

    const constraints = [];

    // ジョイント
    for (const joint of this.joints) {
      // ジョイントに拘束を生成するメソッドを用意した方がいい
      // ジョイントの種類を増やすことがあれば直す
      const cnst = new DistanceConstraint(dt, joint);
      if (cnst.goalVel) {
        constraints.push(cnst);
      }
    }

    // 衝突検出
    this.contacts = [];
    const gNorm = Math.sqrt(this.gravity.normSq())
    const shapes = this.bodies.map((b) => b.movedShape());
    for (let i = 0; i < this.bodies.length; i++) {
      const body1 = this.bodies[i];
      const shape1 = shapes[i];
      if (!shape1) { continue; }
      for (let j = i + 1; j < this.bodies.length; j++) {
        const body2 = this.bodies[j];
        const shape2 = shapes[j];
        if (!shape2) { continue; }
        if (body1.frozen && body2.frozen) { continue; }
        if (body1.noCollide.has(body2.id)) { continue; }

        const col = Collision.polygon_polygon(shape1, shape2);
        if (col === undefined) { continue; }

        for (let i = 0; i < col.points.length; i++) {
          const contact = new Contact(dt, gNorm, body1, body2, col, i);
          constraints.push(contact);
          this.contacts.push(contact);
        }
      }
    }

    // 衝突応答
    const loop = 100;
    for (let i = 0; i < loop; i++) {
      for (const cnst of constraints) {
        cnst.solve();
      }
    }
  }

}