#pragma once

#include <vector>

namespace planner {

    class PolynomialFunction {
    private:
        /**
         * The coefficients of the polynomial, ordered by degree -- i.e.,
         * coefficients[0] is the constant term and coefficients[n] is the
         * coefficient of x^n where n is the degree of the polynomial.
         */
        std::vector<double> coefficients;

        /**
         * Uses Horner's Method to evaluate the polynomial with the given coefficients at
         * the argument.
         *
         * @param coefficients Coefficients of the polynomial to evaluate.
         * @param argument Input value.
         * @return the value of the polynomial.
         * @throws NoDataException if {@code coefficients} is empty.
         * @throws NullArgumentException if {@code coefficients} is {@code null}.
         */
        static double evaluate(const std::vector<double>& coefficients, double argument);

        /**
         * Returns the coefficients of the derivative of the polynomial with the given coefficients.
         *
         * @param coefficients Coefficients of the polynomial to differentiate.
         * @return the coefficients of the derivative or {@code null} if coefficients has length 1.
         * @throws NoDataException if {@code coefficients} is empty.
         * @throws NullArgumentException if {@code coefficients} is {@code null}.
         */
        static std::vector<double> differentiate(const std::vector<double>& coefficients);
    public:
        PolynomialFunction(std::vector<double> c);

        /**
         * Compute the value of the function for the given argument.
         * <p>
         *  The value returned is </p><p>
         *  {@code coefficients[n] * x^n + ... + coefficients[1] * x  + coefficients[0]}
         * </p>
         *
         * @param x Argument for which the function value should be computed.
         * @return the value of the polynomial at the given point.
         *
         * @see org.apache.commons.math4.analysis.UnivariateFunction#value(double)
         */
        double value(double x) const;

        /**
         * Add a polynomial to the instance.
         *
         * @param p Polynomial to add.
         * @return a new polynomial which is the sum of the instance and {@code p}.
         */
        PolynomialFunction add(const PolynomialFunction& p) const;

        /**
         * Multiply the instance by a polynomial.
         *
         * @param p Polynomial to multiply by.
         * @return a new polynomial equal to this times {@code p}
         */
        PolynomialFunction multiply(const PolynomialFunction& p) const;

        /**
         * Returns the derivative as a {@link PolynomialFunction}.
         *
         * @return the derivative polynomial.
         */
        PolynomialFunction derivative() const;
    };

}