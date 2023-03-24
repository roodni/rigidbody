export class Vec2 {
  x: number;
  y: number;

  constructor(x: number, y: number) {
    this.x = x;
    this.y = y;
  }

  dot(v: Vec2) {
    return this.x*v.x +this.y*v.y;
  }

  cross(v: Vec2) {
    return this.x*v.y - this.y*v.x;
  }

  times(a: number) {
    return new Vec2(this.x*a, this.y*a);
  }

  add(v: Vec2) {
    return new Vec2(this.x +v.x, this.y +v.y);
  }

  // this -> dest
  to(dest: Vec2) {
    return dest.add(this.times(-1));
  }

  rot270() {
    return new Vec2(this.y, -this.x)
  }

  normalize() {
    const l = Math.sqrt(this.x**2 + this.y**2);
    return new Vec2(this.x/l, this.y/l);
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
    return new Vec2(this.a*v.x +this.b*v.y, this.c*v.x +this.d*v.y);
  }

  static rotate(t: number) {
    const c = Math.cos(t);
    const s = Math.sin(t);
    return new Mat2(c, -s, s, c);
  }
}

export class Polygon {
  vs: Vec2[]; // 正方向の回り
  axes: Vec2[];
  constructor(vs: Vec2[]) {
    this.vs = vs;
    this.axes = [];
    for (let i = 0; i < vs.length; i++) {
      const v1 = vs[i];
      const v2 = vs[(i + 1) % vs.length];
      const n = v1.to(v2).rot270().normalize();
      this.axes.push(n);
    }
  }

  projection(axis: Vec2): [number, number] {
    let min = Infinity;
    let max = -Infinity;
    for (const v of this.vs) {
      const p = axis.dot(v);
      min = Math.min(min, p);
      max = Math.max(max, p);
    }
    return [min, max];
  }

  static regular(n: number, center: Vec2, radius: number, rot=0) {
    const l: Vec2[] = [];
    const base = new Vec2(radius, 0);
    for (let i = 0; i < n; i++) {
       const v = Mat2.rotate(rot + Math.PI*2*i/n).mulvec(base).add(center);
       l.push(v);
    }
    return new Polygon(l);
  }
}

export class Collision {
  normal: Vec2;
  depth: number;
  points1: Vec2[];
  points2: Vec2[];
  constructor(normal: Vec2, depth: number, points1: Vec2[], points2: Vec2[]) {
    this.normal = normal;
    this.depth = depth;
    this.points1 = points1;
    this.points2 = points2
  }

  static polygon_polygon(poly1: Polygon, poly2: Polygon) {
    function shallowestEdge(polyAxis: Polygon, polyOther: Polygon) {
      // 分離軸判定の片側
      // 最も貫通の浅い軸について貫通深度・辺番号
      // 分離軸が存在したら打ち切る
      let minDepth = Infinity;
      let minAxisI = 0;
      for (let i = 0; i < polyAxis.axes.length; i++) {
        const axis = polyAxis.axes[i];
        const [l1, r1] = polyAxis.projection(axis);
        const [l2, r2] = polyOther.projection(axis);
        const depth = r1 - l2;
        if (depth < 0 || (r1 -l1) +(r2 -l2) < depth) {
          return undefined;
        }
        if (depth < minDepth) {
          minDepth = depth;
          minAxisI = i;
        }
      }
      return {
        depth: minDepth,
        axisI: minAxisI
      };
    }

    const se1 = shallowestEdge(poly1, poly2);
    if (se1 === undefined) return undefined;
    const se2 = shallowestEdge(poly2, poly1);
    if (se2 === undefined) return undefined;

    let reversed = false;
    let se;
    if (se1.depth <= se2.depth) {
      se = se1;
    } else {
      reversed = true;
      se = se2;
      [poly1, poly2] = [poly2, poly1];
    }
    const {depth, axisI} = se;
    const normal = poly1.axes[axisI];
    const edgePerpPos = poly1.vs[axisI].dot(normal);  // 法線に沿った辺の位置
    // 以下、poly1は衝突軸の元になったポリゴン、poly2はもう一方のポリゴン

    // もう一方のポリゴンから頂点をめり込みが大きい順に1つまたは2つ持ってくる
    const vs2 = poly2.vs.map((v) => {
      const p = v.dot(normal);
      return {p, v};
    });
    vs2.sort((a, b) => a.p - b.p);

    // 衝突点を算出する
    const points1 = [];
    const points2 = [];
    if (edgePerpPos < vs2[1].p) {
      // 1点衝突
      points1.push(vs2[0].v.add(normal.times(depth)))
      points2.push(vs2[0].v);
    } else {
      // 2点衝突
      const edge = normal.rot270();
      // 衝突候補点を接触面に沿った位置で並べて、内側の2つを取り出す
      const paraPoss = [
        poly1.vs[axisI],
        poly1.vs[(axisI + 1) % poly1.vs.length],
        vs2[0].v,
        vs2[1].v,
      ].map((v) => edge.dot(v));
      paraPoss.sort((a, b) => a - b);
      for (const paraPos of [paraPoss[1], paraPoss[2]]) {
        const v = edge.times(paraPos);
        points1.push(normal.times(edgePerpPos).add(v));
        points2.push(normal.times(edgePerpPos -depth).add(v));
      }
    }

    // おわり
    if (reversed) {
      return new Collision(normal.times(-1), depth, points2, points1);
    } else {
      return new Collision(normal, depth, points1, points2);
    }
  }
}