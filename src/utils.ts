export class Vec2 {
  x: number;
  y: number;

  private constructor(x: number, y: number) {
    this.x = x;
    this.y = y;
  }
  static c(x: number, y: number) {
    if (!isFinite(x) || !isFinite(y)) {
      throw new Error('not finite');
    }
    return new Vec2(x, y);
  }
  static zero() {
    return new Vec2(0, 0);
  }

  copy() {
    return new Vec2(this.x, this.y);
  }
  copyMut(v: Vec2) {
    this.x = v.x;
    this.y = v.y;
    return this;
  }

  update(x: number, y: number) {
    this.x = x;
    this.y = y;
  }
  zero() {
    this.x = 0;
    this.y = 0;
    return this;
  }

  dot(v: Vec2) {
    return this.x*v.x +this.y*v.y;
  }
  cross(v: Vec2) {
    return this.x*v.y -this.y*v.x;
  }

  normSq() {
    return this.x**2 +this.y**2;
  }
  norm() {
    return Math.sqrt(this.normSq());
  }

  normalize() {
    const l = this.norm();
    return Vec2.c(this.x/l, this.y/l);
  }

  /** 正規化、ただしゼロベクトルに対して特別な値を返す */
  safeNormalize<T>(zero: T): Vec2 | T {
    const l = this.norm();
    if (l < Number.EPSILON) {
      return zero;
    } else {
      return Vec2.c(this.x/l, this.y/l);
    }
  }

  rot90() {
    return new Vec2(-this.y, this.x);
  }
  rotMut90() {
    const x = this.x;
    this.x = -this.y;
    this.y = x;
    return this;
  }

  rot270() {
    return new Vec2(this.y, -this.x);
  }
  rotMut270() {
    const x = this.x;
    this.x = this.y;
    this.y = -x;
    return this;
  }

  reverse() {
    return Vec2.c(-this.x, -this.y);
  }
  reverseMut() {
    this.x = -this.x;
    this.y = -this.y;
    return this;
  }

  times(a: number) {
    return Vec2.c(this.x*a, this.y*a);
  }
  timesMut(a: number) {
    this.x *= a;
    this.y *= a;
    return this;
  }

  add(v: Vec2) {
    return Vec2.c(this.x +v.x, this.y +v.y);
  }
  addMut(v: Vec2, scale = 1) {
    this.x += v.x * scale;
    this.y += v.y * scale;
    return this;
  }

  sub(v: Vec2) {
    return Vec2.c(this.x -v.x, this.y -v.y);
  }
  subMut(v: Vec2) {
    this.x -= v.x;
    this.y -= v.y;
    return this;
  }
  to(dest: Vec2) {
    return dest.sub(this);
  }

  toTuple(): [number, number] {
    return [this.x, this.y];
  }
}

export class Mat2 {
  // [a b]
  // [c d]
  a: number;
  b: number;
  c: number;
  d: number;

  constructor(a: number, b: number, c: number, d: number) {
    this.a = a; this.b = b;
    this.c = c; this.d = d;
  }

  mulvec(v: Vec2) {
    return Vec2.c(this.a*v.x +this.b*v.y, this.c*v.x +this.d*v.y);
  }

  static rotate(t: number) {
    const c = Math.cos(t);
    const s = Math.sin(t);
    return new Mat2(c, -s, s, c);
  }
}

export function randInt(a: number, b: number) {
  const n = b - a + 1;
  const r = Math.floor(Math.random() * n);
  return a + r;
}
export function rand(a: number, b: number) {
  return a + Math.random() * (b - a);
}

export function shuffle(a: unknown[]) {
  for (let i = a.length - 1; i >= 1; i--) {
    const j = randInt(0, i);
    [a[i], a[j]] = [a[j], a[i]];
  }
}