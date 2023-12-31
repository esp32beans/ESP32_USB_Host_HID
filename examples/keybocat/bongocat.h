/*
 * MIT License
 *
 * Copyright (c) 2023 esp32beans@gmail.com
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
/*
 * Call bongo_setup() from the Arduino setup(). bongo_setup starts
 * a task to display images.
 *   void bongo_setup(void);
 *
 * The following functions display bongo images.
 * However these functions can take up 80-100 ms to return. Do not call them
 * from inside a callback function such as USB host. These slow functions
 * will cause the USB host to drop data.
 *   void bongo_idle(void)  // Both paws up
 *   void bongo_left(void)  // Left paw down
 *   void bongo_right(void) // Right paw down
 *   void bongo_loop(void)  // Cycle through the above images 1 per call
 *
 * bongo_wake() sends a notification to the bongo task to display the next
 * bongo image. This function returns without delay so can be called from
 * callback functions.
 *   void bongo_wake(void);
 *
 */

void bongo_setup(void);
void bongo_idle(void);
void bongo_left(void);
void bongo_right(void);
void bongo_loop(void);
void bongo_wake(void);
