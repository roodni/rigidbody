import * as utils from './utils';
import {Vec2, Mat2} from './utils';
import {Shape, Collision} from './shape';

export class RigidBody {
  static idPool = 0;

  id: number;
  shape: Shape | undefined;
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

  constructor(shape: Shape | undefined, mass: number, inertia: number) {
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
    return this.shape.euclidean(this.angle, this.pos);
  }

  /** ワールド座標のポリゴンからインスタンスを生成する */
  static fromShape(shape: Shape, density = 1.0) {
    const {
      area,
      inertia: areaInertia,
      center
    } = shape.areas();

    const mass = area * density;
    const inertia = areaInertia * density;

    shape = shape.euclidean(0, center.reverse());
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


interface Constraint {
  warmStart(): void;
  solve(): void;
}

export class Contact implements Constraint {
  body1: RigidBody;
  body2: RigidBody;
  pointi: number;
  normal: Vec2;
  r1: Vec2;
  r2: Vec2;
  impulse: Vec2;
  goalVel: number;

  invMassN: number;
  invMassT: number;
  invMassNT: number;

  isFrictionStatic: boolean;

  constructor(dt: number, body1: RigidBody, body2: RigidBody, collision: Collision, pointi: number, prev?: Contact) {
    const GRAVITY = 9.8;
    this.body1 = body1;
    this.body2 = body2;
    this.pointi = pointi;
    const normal = this.normal = collision.normal;
    const tangent = normal.rot90();
    const [point1, point2, depth] = collision.points[pointi];
    const point = point1.add(point2).times(1/2);
    const r1 = this.r1 = body1.pos.to(point);
    const r2 = this.r2 = body2.pos.to(point);
    this.impulse = prev?.impulse ?? Vec2.ZERO;
    // 目標となる法線方向の相対速度の算出
    const vel12 = body2.velAtLocal(r2).sub(body1.velAtLocal(r1)).dot(normal);
    let restitution = body1.restitution * body2.restitution;
    // 相対速度が十分小さいとき反発係数をゼロにする
    if (Math.abs(vel12) < GRAVITY * dt * restitution * 10) {
      restitution = 0;
    }
    const velReaction = -restitution * vel12;
    const slop = GRAVITY * dt * dt * 3;
    const velError = Math.min(depth - slop, slop) / dt;
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

  warmStart() {
    this.body1.addImpulseLocal(this.r1, this.impulse.reverse());
    this.body2.addImpulseLocal(this.r2, this.impulse);
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
    this.impulse = Vec2.ZERO;

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
    this.body1.addImpulseLocal(this.r1, this.impulse.reverse());
    this.body2.addImpulseLocal(this.r2, this.impulse);
  }
}

class ContactDictionary {
  map: Map<number, Map<number, Contact[]>>;
    // id -> id -> pointi -> Contact
    // オブジェクトの順番にsensitive
  constructor() {
    this.map = new Map();
  }
  clear() {
    this.map.clear();
  }
  find(id1: number, id2: number) {
    return this.map.get(id1)?.get(id2);
  }
  add(cont: Contact) {
    let map2 = this.map.get(cont.body1.id);
    if (!map2) {
      map2 = new Map();
      this.map.set(cont.body1.id, map2);
    }
    let arr = map2.get(cont.body2.id);
    if (!arr) {
      arr = [];
      map2.set(cont.body2.id, arr);
    }
    arr[cont.pointi] = cont;
  }
}


class PinConstraint implements Constraint {
  body1: RigidBody;
  body2: RigidBody;
  point: Vec2;
  goalVel: Vec2;

  invMassX: number;
  invMassY: number;
  invMassXY: number;

  impulse: Vec2;

  constructor(body1: RigidBody, body2: RigidBody, point: Vec2, goalVel: Vec2, prev?: PinConstraint) {
    const b1 = this.body1 = body1;
    const b2 = this.body2 = body2;
    this.point = point;
    this.goalVel = goalVel;
    const r1 = b1.pos.to(point);
    const r2 = b2.pos.to(point);

    this.invMassX = b1.invMass +b2.invMass +b1.invInertia*r1.y**2 +b2.invInertia*r2.y**2;
    this.invMassY = b1.invMass +b2.invMass +b1.invInertia*r1.x**2 +b2.invInertia*r2.x**2;
    this.invMassXY = -b1.invInertia*r1.x*r1.y -b2.invInertia*r2.x*r2.y;

    this.impulse = prev?.impulse ?? Vec2.ZERO;
  }

  warmStart() {
    this.body1.addImpulse(this.point, this.impulse.reverse());
    this.body2.addImpulse(this.point, this.impulse);
  }

  solve() {
    const vel1 = this.body1.velAt(this.point);
    const vel2 = this.body2.velAt(this.point);
    const vel12 = vel2.sub(vel1);
    const velDiff = this.goalVel.sub(vel12);

    const ix = Vec2.c(this.invMassY, -this.invMassXY).dot(velDiff);
    const iy = Vec2.c(-this.invMassXY, this.invMassX).dot(velDiff);
    const impulse = Vec2.c(ix, iy).times(1/(this.invMassX*this.invMassY -this.invMassXY**2));
    this.body1.addImpulse(this.point, impulse.reverse());
    this.body2.addImpulse(this.point, impulse);
    this.impulse = this.impulse.add(impulse);
  }
}

export class PinJoint {
  constraint?: PinConstraint;

  constructor(
    public body1: RigidBody,
    public body2: RigidBody,
    public r1: Vec2,
    public r2: Vec2,
    public noCollide: boolean
  ) {}

  pos1() {
    const rot1 = Mat2.rotate(this.body1.angle);
    return rot1.mulvec(this.r1).add(this.body1.pos);
  }
  pos2() {
    const rot2 = Mat2.rotate(this.body2.angle);
    return rot2.mulvec(this.r2).add(this.body2.pos);
  }

  updateConstraint(dt: number) {
    const p1 = this.pos1();
    const p2 = this.pos2();
    const p12 = p1.to(p2);

    const d12 = p12.norm();
    const center = p1.add(p12.times(0.5));
    const goalVel = p12.times(-d12 / dt);

    this.constraint = new PinConstraint(this.body1, this.body2, center, goalVel, this.constraint);
    return this.constraint;
  }

  static pin(body1: RigidBody, body2: RigidBody, point: Vec2, noCollide=true) {
    const rot1 = Mat2.rotate(-body1.angle);
    const rot2 = Mat2.rotate(-body2.angle);
    const r1 = rot1.mulvec(body1.pos.to(point));
    const r2 = rot2.mulvec(body2.pos.to(point));
    return new PinJoint(body1, body2, r1, r2, noCollide);
  }
}


export class World {
  gravity = Vec2.c(0, 9.8);
  ether: RigidBody;
  bodies: RigidBody[] = [];
  shapes: (Shape | undefined)[] = [];
  joints: PinJoint[] = [];
  contacts: Contact[] = [];

  contactDict = new ContactDictionary();

  constructor() {
    this.ether = RigidBody.ether();
    this.addBody(this.ether);
  }

  addBody(body: RigidBody) {
    this.bodies.push(body);
  }

  addJoint(joint: PinJoint) {
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
    if (dt <= 0) { return; }

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
      const cnst = joint.updateConstraint(dt);
      if (cnst) {
        constraints.push(cnst);
      }
    }

    // 衝突検出
    this.contacts = [];
    this.shapes = this.bodies.map((b) => b.movedShape());
    for (let i = 0; i < this.bodies.length; i++) {
      const body1 = this.bodies[i];
      const shape1 = this.shapes[i];
      if (!shape1) { continue; }
      for (let j = i + 1; j < this.bodies.length; j++) {
        const body2 = this.bodies[j];
        const shape2 = this.shapes[j];
        if (!shape2) { continue; }
        if (body1.frozen && body2.frozen) { continue; }
        if (body1.noCollide.has(body2.id)) { continue; }

        const col = shape1.collide(shape2);
        if (col === undefined) { continue; }

        const prevContacts = this.contactDict.find(body1.id, body2.id);
        for (let i = 0; i < col.points.length; i++) {
          const contact = new Contact(dt, body1, body2, col, i, prevContacts?.at(i));
          constraints.push(contact);
          this.contacts.push(contact);
        }
      }
    }
    this.contactDict.clear();
    for (const cont of this.contacts) {
      this.contactDict.add(cont);
    }

    // 衝突応答
    utils.shuffle(constraints);
    for (const cnst of constraints) {
      cnst.warmStart();
    }
    const loop = 30;
    for (let i = 0; i < loop; i++) {
      for (const cnst of constraints) {
        cnst.solve();
      }
    }
  }

}