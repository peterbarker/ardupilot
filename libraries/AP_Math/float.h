#pragma once

static const float NaNf = nanf("0x4152");

/*
 * Check whether two floats are equal
 */
template <typename Arithmetic1, typename Arithmetic2>
typename std::enable_if<std::is_integral<typename std::common_type<Arithmetic1, Arithmetic2>::type>::value ,bool>::type
is_equal(const Arithmetic1 v_1, const Arithmetic2 v_2);

template <typename Arithmetic1, typename Arithmetic2>
typename std::enable_if<std::is_floating_point<typename std::common_type<Arithmetic1, Arithmetic2>::type>::value, bool>::type
is_equal(const Arithmetic1 v_1, const Arithmetic2 v_2);

/* 
 * @brief: Check whether a float is zero
 */
template <typename T>
inline bool is_zero(const T fVal1) {
    static_assert(std::is_floating_point<T>::value || std::is_base_of<T,AP_Float>::value,
                  "Template parameter not of type float");
    return is_zero(static_cast<float>(fVal1));
}

/* 
 * @brief: Check whether a float is greater than zero
 */
template <typename T>
inline bool is_positive(const T fVal1) {
    static_assert(std::is_floating_point<T>::value || std::is_base_of<T,AP_Float>::value,
                  "Template parameter not of type float");
    return (static_cast<float>(fVal1) >= FLT_EPSILON);
}


/* 
 * @brief: Check whether a float is less than zero
 */
template <typename T>
inline bool is_negative(const T fVal1) {
    static_assert(std::is_floating_point<T>::value || std::is_base_of<T,AP_Float>::value,
                  "Template parameter not of type float");
    return (static_cast<float>(fVal1) <= (-1.0 * FLT_EPSILON));
}

/*
 * @brief: Check whether a double is greater than zero
 */
inline bool is_positive(const double fVal1) {
    return (fVal1 >= static_cast<double>(FLT_EPSILON));
}

/*
 * @brief: Check whether a double is less than zero
 */
inline bool is_negative(const double fVal1) {
    return (fVal1 <= static_cast<double>((-1.0 * FLT_EPSILON)));
}
