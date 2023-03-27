import p5 from 'p5';
import { Vec2 } from './utils';
import { Polygon } from './shape';
import { RigidBody, Contact, World } from './physics';

export class Draw {
  p: p5;
  meterToPx: number;
  origin: Vec2;
  constructor(p: p5, meterToPx: number, origin=Vec2.ZERO) {
    this.p = p;
    this.meterToPx = meterToPx;
    this.origin = origin;
  }

  posToPx(v: Vec2) {
    return this.origin.to(v).times(this.meterToPx);
  }

  drawPolygon(poly: Polygon) {
    this.p.beginShape();
    for (const v of poly.vertices) {
      this.p.vertex(...this.posToPx(v).toTuple());
    }
    this.p.endShape(this.p.CLOSE);
  }

  drawBody(body: RigidBody) {
    const p = this.p;
    p.colorMode(p.HSB, 1.0);
    let color = p.color(0.0, 0.2, 1.0);
    if (body.frozen) {
      color = p.color(0.6);
    }

    p.strokeWeight(3);
    p.stroke(color);
    p.noFill();
    this.drawPolygon(body.movedShape());
    p.circle(...this.posToPx(body.pos).toTuple(), 5);
  }

  drawContact(contact: Contact) {
    const p = this.p;
    const pos1 = this.posToPx(contact.body1.pos.add(contact.r1));
    const pos2 = this.posToPx(contact.body2.pos.add(contact.r2));

    p.colorMode(p.RGB, 255);
    let color = p.color(255, 0, 0);
    if (contact.isFrictionStatic) {
      color = p.color(0, 255, 0); // 静止摩擦なら緑
    }
    p.fill(color);
    p.noStroke();

    const r = 10;
    p.circle(...pos1.toTuple(), r);
    p.circle(...pos2.toTuple(), r);
  }

  drawWorld(world: World) {
    for (const b of world.bodies) {
      this.drawBody(b);
    }
    for (const c of world.constraints) {
      this.drawContact(c);
    }
  }
}