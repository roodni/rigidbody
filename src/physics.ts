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
  pos = Vec2.zero();
  vel = Vec2.zero();
  acc = Vec2.zero();
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
    this.vel = Vec2.zero();
    this.acc = Vec2.zero();
  }
  unfreeze() {
    this.frozen = false;
    this.invMass = 1 / this.mass;
    this.invInertia = 1 / this.invInertia;
  }

  addForceLocal(r: Vec2, force: Vec2) {
    this.acc.addMut(force, this.invMass);
    this.aacc += r.cross(force) * this.invInertia;
  }
  addForce(point: Vec2, force: Vec2) {
    const r = this.pos.to(point);
    this.addForceLocal(r, force);
  }

  addImpulseLocal(r: Vec2, impulse: Vec2) {
    this.vel.addMut(impulse, this.invMass);
    this.avel += r.cross(impulse) * this.invInertia;
  }

  velAtLocal(r: Vec2, dest?: Vec2) {
    if (!dest) {
      dest = Vec2.zero();
    } else {
      dest.zero();
    }
    r.rotMut90();
    dest.addMut(this.vel).addMut(r, this.avel);
    r.rotMut270();
    return dest;
  }
  velAt(point: Vec2) {
    const r = this.pos.to(point);
    return this.velAtLocal(r);
  }

  static velRelative(b1: RigidBody, b2: RigidBody, r1: Vec2, r2: Vec2, dest: Vec2) {
    r1.rotMut90();
    r2.rotMut90();
    dest.zero()
      .addMut(b2.vel).addMut(r2, b2.avel)
      .subMut(b1.vel).addMut(r1, -b1.avel);
    r1.rotMut270();
    r2.rotMut270();
    return dest;
  }

  // accAtLocal(r: Vec2) {
  //   return this.acc.add(r.rot90().times(this.aacc)).add(r.reverse().times(this.avel**2));
  // }

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
  tangent: Vec2;
  r1: Vec2;
  r2: Vec2;
  impulse: Vec2;
  goalVel: number;

  invMassN: number;
  invMassT: number;
  invMassNT: number;
  temp: Vec2;  // 計算領域

  isFrictionStatic: boolean;

  constructor(dt: number, body1: RigidBody, body2: RigidBody, collision: Collision, pointi: number, prev?: Contact) {
    this.body1 = body1;
    this.body2 = body2;
    this.pointi = pointi;
    const normal = this.normal = collision.normal;
    const [point1, point2, depth] = collision.points[pointi];
    if (prev) {
      // 前回の拘束のVec2を使い回す
      // prevで指定されたオブジェクトが他のprevにならないという仮定が必要
      this.impulse = prev.impulse;

      this.tangent = prev.tangent.copyMut(normal).rotMut90();
      this.r1 = prev.r1.copyMut(point1);
      this.r2 = prev.r2;
      this.temp = prev.temp;
    } else {
      this.impulse = Vec2.zero();

      this.tangent = normal.rot90();
      this.r1 = point1.copy();
      this.r2 = Vec2.zero();
      this.temp = Vec2.zero();
    }

    const tangent = this.tangent;
    this.r1.addMut(point2).timesMut(0.5); // r1はこの時点で中点を指す
    const r2 = this.r2.copyMut(this.r1).subMut(body2.pos);
    const r1 = this.r1.subMut(body1.pos);

    // 目標となる法線方向の相対速度の算出
    const vel12 = RigidBody.velRelative(body1, body2, r1, r2, this.temp).dot(normal);
    let restitution = body1.restitution * body2.restitution;
    // const acc12 =
    //   body2.acc.add(r2.rot90().times(body2.aacc))
    //   .sub(body1.acc.add(r1.rot90().times(body1.aacc)));
    r1.rotMut90(); r2.rotMut90();
    const acc12 = this.temp.zero()
      .addMut(body2.acc).addMut(r2, body2.aacc)
      .subMut(body1.acc).addMut(r1, -body1.aacc);
    r1.rotMut270(); r2.rotMut270();
    const velAccCorrection = Math.min(0, normal.dot(acc12) * dt);
    const velReaction = Math.max(
      0, -restitution*vel12 + velAccCorrection
    );
    const slop = 9.8 * dt * dt * 3;
    const velError = Math.min(depth - slop, slop) / dt;
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

    this.isFrictionStatic = true;
  }

  warmStart() {
    this.impulse.reverseMut();
    this.body1.addImpulseLocal(this.r1, this.impulse);
    this.impulse.reverseMut();
    this.body2.addImpulseLocal(this.r2, this.impulse);
  }

  solve() {
    const r1 = this.r1;
    const r2 = this.r2;
    const normal = this.normal;
    const tangent = this.tangent;
    const invMassN = this.invMassN;
    const invMassT = this.invMassT;
    const invMassNT = this.invMassNT;

    // 前回与えた撃力を打ち消す
    this.body1.addImpulseLocal(r1, this.impulse);
    this.impulse.reverseMut();
    this.body2.addImpulseLocal(r2, this.impulse);
    this.impulse.zero();

    const vel12 = RigidBody.velRelative(this.body1, this.body2, r1, r2, this.temp);
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
      impulseN = velDiff / invMass;
      impulseT = sign * friction * impulseN;
    }

    this.impulse.addMut(tangent, impulseT).addMut(normal, impulseN);
    this.impulse.reverseMut();
    this.body1.addImpulseLocal(this.r1, this.impulse);
    this.impulse.reverseMut();
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
  r1: Vec2;
  r2: Vec2;
  goalVel: Vec2;

  invMassX: number;
  invMassY: number;
  invMassXY: number;

  impulse: Vec2;
  velDiff: Vec2;

  constructor(body1: RigidBody, body2: RigidBody, point: Vec2, goalVel: Vec2, prev?: PinConstraint) {
    const b1 = this.body1 = body1;
    const b2 = this.body2 = body2;
    this.goalVel = goalVel;
    const r1 = this.r1 = b1.pos.to(point);
    const r2 = this.r2 = b2.pos.to(point);

    this.invMassX = b1.invMass +b2.invMass +b1.invInertia*r1.y**2 +b2.invInertia*r2.y**2;
    this.invMassY = b1.invMass +b2.invMass +b1.invInertia*r1.x**2 +b2.invInertia*r2.x**2;
    this.invMassXY = -b1.invInertia*r1.x*r1.y -b2.invInertia*r2.x*r2.y;

    this.impulse = prev?.impulse ?? Vec2.zero();
    this.velDiff = Vec2.zero();
  }

  warmStart() {
    this.impulse.reverseMut();
    this.body1.addImpulseLocal(this.r1, this.impulse);
    this.impulse.reverseMut();
    this.body2.addImpulseLocal(this.r2, this.impulse);
  }

  solve() {
    RigidBody.velRelative(this.body1, this.body2, this.r1, this.r2, this.velDiff);
    const velDiff = this.velDiff.reverseMut().addMut(this.goalVel);

    // 以前の撃力のバックアップ
    const ix = this.impulse.x;
    const iy = this.impulse.y;


    const impulse = this.impulse;
    impulse.x =  this.invMassY*velDiff.x -this.invMassXY*velDiff.y;
    impulse.y = -this.invMassXY*velDiff.x +this.invMassX*velDiff.y;
    impulse.timesMut(1 / (this.invMassX*this.invMassY -this.invMassXY**2))

    impulse.reverseMut();
    this.body1.addImpulseLocal(this.r1, impulse);
    impulse.reverseMut();
    this.body2.addImpulseLocal(this.r2, impulse);
    impulse.x += ix;
    impulse.y += iy;
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


class Aabb {
  body: RigidBody;
  shape: Shape;
  mins: number[];
  maxs: number[];

  constructor(body: RigidBody, shape: Shape) {
    this.body = body;
    this.shape = shape;
    const [x1, x2] = shape.projection(Vec2.c(1, 0));
    const [y1, y2] = shape.projection(Vec2.c(0, 1));
    this.mins = [x1, y1];
    this.maxs = [x2, y2];
  }

  overlap(axis: number, aabb: Aabb) {
    return this.mins[axis] <= aabb.maxs[axis] && this.mins[axis] <= aabb.maxs[axis];
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
  deletionSet = new Set<number>();  // 削除する物体のidを入れる

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

  deleteBody(body: RigidBody) {
    this.deletionSet.add(body.id);
  }

  step(dt: number) {
    // 削除予約の実行
    let bodyCount = 0;
    for (let i = 0; i < this.bodies.length; i++) {
      const body = this.bodies[i];
      if (this.deletionSet.has(body.id)) {
        // 他のbodyのnoCollideから削除した方が良いのだが、面倒なので無視
        // ジョイント消去も線形探策でいいや
        for (let j = 0; j < this.joints.length; j++) {
          const joint = this.joints[j];
          if (joint.body1 === body || joint.body2 === body) {
            this.joints[j--] = this.joints[this.joints.length - 1];
            this.joints.length--;
          }
        }
      } else {
        this.bodies[bodyCount++] = body;
      }
    }
    this.bodies.length = bodyCount;
    this.deletionSet.clear();

    if (dt <= 0) { return; }

    // 重力
    for (const b of this.bodies) {
      b.addForce(b.pos, this.gravity.times(b.mass));
    }

    // 積分
    for (const b of this.bodies) {
      b.pos = b.pos.add(b.vel.times(dt));
      b.vel = b.vel.add(b.acc.times(dt));
      b.angle += b.avel * dt;
      b.avel += b.aacc * dt;
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
    const aabbs = [];
    for (let i = 0; i < this.bodies.length; i++) {
      const s = this.shapes[i];
      if (!s) { continue; }
      const b = this.bodies[i];
      aabbs.push(new Aabb(b, s));
    }
    const axis = 0;
    aabbs.sort((a, b) => a.mins[axis] - b.mins[axis]);
    for (let i = 0; i < aabbs.length; i++) {
      const aabb1 = aabbs[i];
      for (let j = i + 1; j < aabbs.length; j++) {
        const aabb2 = aabbs[j];
        if (aabb1.maxs[axis] < aabb2.mins[axis]) { break; }
        let b1 = aabb1.body;
        let b2 = aabb2.body;
        if (b1.frozen && b2.frozen) { continue; }
        if (b1.noCollide.has(b2.id)) { continue; }
        if (!aabb1.overlap(1 - axis, aabb2)) { continue; }
        const col = aabb1.shape.collide(aabb2.shape);
        if (col) {
          if (b1.id > b2.id) {
            col.reverse();
            [b1, b2] = [b2, b1];
          }
          const prevContacts = this.contactDict.find(b1.id, b2.id);
          for (let i = 0; i < col.points.length; i++) {
            const contact = new Contact(dt, b1, b2, col, i, prevContacts?.at(i));
            constraints.push(contact);
            this.contacts.push(contact);
          }
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

    // 加速度のクリア
    for (const b of this.bodies) {
      b.acc = Vec2.zero();
      b.aacc = 0;
    }
  }

  findBody(hitShape: Shape, pred = (b: RigidBody) => true) {
    for (let i = this.shapes.length - 1; i >= 0; i--) {
      const body = this.bodies[i];
      const shape = this.shapes[i];
      if (!shape || !pred(body)) { continue; }
      if (hitShape.collide(shape)) {
        return body;
      }
    }
    return undefined;
  }
}