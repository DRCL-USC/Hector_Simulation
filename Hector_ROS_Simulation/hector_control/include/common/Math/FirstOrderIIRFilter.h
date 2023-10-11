/*
MIT License

Copyright (c) 2019 MIT Biomimetics Robotics Lab

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

/*! @file FirstOrderIIRFilter.h
 *  @brief A simple first order filter
 *
 *
 */

#ifndef PROJECT_FIRSTORDERIIRFILTER_H
#define PROJECT_FIRSTORDERIIRFILTER_H

#include <cmath>

/*!
 * First Order Filter
 * @tparam T : type of the data to be filtered
 * @tparam T2 : floating point type for the cutoff/sample frequencies, gain
 */
template <typename T, typename T2>
class FirstOrderIIRFilter {
 public:
  /*!
   * Create a new first order filter
   * @param cutoffFrequency : cutoff frequency of filter
   * @param sampleFrequency : sample frequency (rate at which update is called)
   * @param initialialValue : initial value stored in the filter
   */
  FirstOrderIIRFilter(T2 cutoffFrequency, T2 sampleFrequency, T& initialValue) {
    _alpha = 1 - std::exp(-2 * M_PI * cutoffFrequency / sampleFrequency);
    _state = initialValue;
  }

  /*!
   * Create a new first order filter
   * @param alpha : filter parameter
   * @param initialValue : initial value
   */
  FirstOrderIIRFilter(T2 alpha, T& initialValue)
      : _state(initialValue), _alpha(alpha) {}

  /*!
   * Update the filter with a new sample
   * @param x : the new sample
   * @return the new state of the filter
   */
  T update(T& x) {
    _state = _alpha * x + (T2(1) - _alpha) * _state;
    return _state;
  }

  /*!
   * Get the value of the filter, without updating
   */
  T get() { return _state; }

  /*!
   * Reset the filter to zero.
   */
  void reset() { _state *= T2(0); }

 private:
  T _state;
  T2 _alpha;
};

#endif  // PROJECT_FIRSTORDERIIRFILTER_H
