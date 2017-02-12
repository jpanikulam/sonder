namespace sonder {
inline constexpr float int_pow(const float base, const unsigned exponent) {
  return (exponent == 0) ? 1 : (base * int_pow(base, exponent - 1));
}
}