#pragma once

#include <finelc/matrix.h>

namespace finelc{

    /**
     * @brief Evaluate the Lagrange polynomial of given order and number at point x.
     * 
     * @param x The point at which to evaluate the polynomial.
     * @param order The order of the Lagrange polynomial.
     * @param number The specific polynomial number to evaluate.
     * 
     * @return double The value of the Lagrange polynomial at point x.
     */
    extern double eval_lagrange_polynomial(double x, int order, int number);

    /**
     * @brief Evaluate the Lagrange polynomial derivative of given order and number at point x.
     * 
     * @param x The point at which to evaluate the polynomial.
     * @param order The order of the Lagrange polynomial.
     * @param number The specific polynomial number to evaluate.
     * 
     * @return double The value of the Lagrange polynomial derivative at point x.
     */
    extern double eval_lagrange_polynomial_derivative(double x, int order, int number);

    /**
     * @brief Evaluate the Lagrange polynomials of given order at point x.
     * 
     * @param x The point at which to evaluate the polynomial.
     * @param order The order of the Lagrange polynomial.
     * 
     * @return Vector The value of all Lagrange polynomials at point x.
     */
    extern Vector eval_lagrange_polynomials(double x, int order);

    /**
     * @brief Evaluate the Lagrange polynomials derivative of given order at point x.
     * 
     * @param x The point at which to evaluate the polynomial derivative.
     * @param order The order of the Lagrange polynomial.
     * 
     * @return Vector The value of all Lagrange polynomials derivative at point x.
     */
    extern Vector eval_lagrange_polynomials_derivatives(double x, int order);
    
} // namespace finelc
