import p5 from 'p5';
import { Vec2, Mat2 } from './utils';
import { Circle, Polygon } from './shape';
import { RigidBody, Contact, World, PinJoint } from './physics';
import config from './config';

export class Draw {
  meterToPx: number;
  origin: Vec2;
  constructor(meterToPx: number, origin=Vec2.ZERO) {
    this.meterToPx = meterToPx;
    this.origin = origin;
  }

  posToPx(v: Vec2) {
    return this.origin.to(v).times(this.meterToPx);
  }
  pxToPos(v: Vec2) {
    return v.times(1 / this.meterToPx).add(this.origin);
  }

  drawPolygon(p: p5, poly: Polygon) {
    p.beginShape();
    for (const v of poly.vertices) {
      p.vertex(...this.posToPx(v).toTuple());
    }
    p.endShape(p.CLOSE);
  }

  drawBody(p: p5, body: RigidBody, color: p5.Color) {
    const shape = body.movedShape();
    if (!shape) { return; }

    p.stroke(color);
    p.noFill();
    if (shape instanceof Polygon) {
      // この分岐はオブジェクト指向の主旨に反しているような気もする
      this.drawPolygon(p, shape);
    } else if (shape instanceof Circle) {
      const c = this.posToPx(shape.center);
      const r = shape.radius * this.meterToPx;
      const rot = Mat2.rotate(body.angle);
      const v = rot.mulvec(Vec2.c(r, 0)).add(c);
      p.circle(c.x, c.y, r * 2);
      p.line(c.x, c.y, v.x, v.y);
    }

    if (config.showCenter) {
      p.noStroke();
      p.fill(color);
      p.circle(...this.posToPx(body.pos).toTuple(), 8);
    }
  }

  drawContact(p: p5, contact: Contact) {
    const pos1 = this.posToPx(contact.body1.pos.add(contact.r1));
    const pos2 = this.posToPx(contact.body2.pos.add(contact.r2));
    const point = pos1.add(pos2).times(1/2);

    p.colorMode(p.RGB, 255);
    let color = p.color(255, 0, 0);
    if (contact.impulse.normSq() == 0) {
      color = p.color(0, 0, 255); // 撃力無しなら青
    } else if (contact.isFrictionStatic) {
      color = p.color(0, 255, 0); // 静止摩擦なら緑
    }
    p.fill(color);
    p.noStroke();

    const r = 6;
    p.circle(...point.toTuple(), r);
    // 法線の可視化
    p.noFill();
    p.stroke(color);
    p.line(point.x, point.y, ...point.add(contact.normal.times(r)).toTuple());
  }

  drawJoint(p: p5, joint: PinJoint) {
    p.colorMode(p.HSB, 1.0);
    const color = p.color(0.5, 0.2, 1.0);
    p.noStroke();
    p.fill(color);

    const p1 = joint.pos1();
    const p2 = joint.pos2();
    p.circle(...this.posToPx(p1.add(p2).times(0.5)).toTuple(), 8)
  }
}