#pragma once

#include "libraidzero/planner/polynomialFunction.hpp"

#include <vector>

namespace planner {
    
    class HermiteInterpolator {
    private:
        static constexpr long long FACTORIALS[] = {
            1L, 1L, 2L,
            6L, 24L, 120L,
            720L, 5040L, 40320L
        };

        /** Sample abscissae. */
        std::vector<double> abscissae;

        /** Top diagonal of the divided differences array. */
        std::vector<std::vector<double>> topDiagonal;

        /** Bottom diagonal of the divided differences array. */
        std::vector<std::vector<double>> bottomDiagonal;
    public:
        /** Add a sample point.
         * <p>
         * This method must be called once for each sample point. It is allowed to
         * mix some calls with values only with calls with values and first
         * derivatives.
         * </p>
         * <p>
         * The point abscissae for all calls <em>must</em> be different.
         * </p>
         * @param x abscissa of the sample point
         * @param value value and derivatives of the sample point
         * (if only one row is passed, it is the value, if two rows are
         * passed the first one is the value and the second the derivative
         * and so on)
         * @exception ZeroException if the abscissa difference between added point
         * and a previous point is zero (i.e. the two points are at same abscissa)
         * @exception MathArithmeticException if the number of derivatives is larger
         * than 20, which prevents computation of a factorial
         */
        void addSamplePoint(double x, const std::vector<double>& value);

        /** Compute the interpolation polynomials.
         * @return interpolation polynomials array
         * @exception NoDataException if sample is empty
         */
        std::vector<PolynomialFunction> getPolynomials() const;

        /** Interpolate value at a specified abscissa.
         * <p>
         * Calling this method is equivalent to call the {@link PolynomialFunction#value(double)
         * value} methods of all polynomials returned by {@link #getPolynomials() getPolynomials},
         * except it does not build the intermediate polynomials, so this method is faster and
         * numerically more stable.
         * </p>
         * @param x interpolation abscissa
         * @return interpolated value
         * @exception NoDataException if sample is empty
         */
        std::vector<double> value(double x) const;
    };

}