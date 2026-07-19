import type { FpRaw, QuatRaw, TransformRaw, Vec3Raw } from "../wasm/gravity";

export const ONE: FpRaw = 1n << 32n;
export const ZERO: Vec3Raw = { x: 0n, y: 0n, z: 0n };
export const IDENTITY: QuatRaw = { x: 0n, y: 0n, z: 0n, w: ONE };

export function decimalToRaw(source: string): FpRaw {
  const match = /^(-?)([0-9]+)(?:\.([0-9]+))?$/.exec(source);
  if (match === null) throw new Error(`invalid canonical decimal: ${source}`);
  const negative = match[1] === "-";
  const whole = BigInt(match[2] ?? "0");
  const fraction = match[3] ?? "";
  const scale = 10n ** BigInt(fraction.length);
  const numerator = (whole * scale + BigInt(fraction === "" ? "0" : fraction)) * ONE;
  let quotient = numerator / scale;
  const remainder = numerator % scale;
  const doubled = remainder * 2n;
  if (doubled > scale || (doubled === scale && (quotient & 1n) === 1n)) quotient += 1n;
  return negative ? -quotient : quotient;
}

export function integerRaw(value: number): FpRaw {
  if (!Number.isSafeInteger(value)) throw new Error("integer is outside exact range");
  return BigInt(value) * ONE;
}

export function vec(x: number, y: number, z: number): Vec3Raw {
  return { x: integerRaw(x), y: integerRaw(y), z: integerRaw(z) };
}

export function transform(position: Vec3Raw = ZERO, orientation: QuatRaw = IDENTITY): TransformRaw {
  return { position, orientation };
}

export function rawToNumber(value: FpRaw): number {
  return Number(value) / 4_294_967_296;
}
