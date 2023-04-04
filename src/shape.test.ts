import { Polygon } from "./shape";
import { Vec2 } from "./utils";

test('正方形の面積と慣性モーメント', () => {
  const r = 1;
  const poly = Polygon.regular(4, Vec2.zero(), r);
  const {area, inertia} = poly.areas();

  expect(area).toBeCloseTo(2.0);
  expect(inertia).toBeCloseTo(2.0 * 2.0 / 6);
});