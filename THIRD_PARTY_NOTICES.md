# Third-Party Notices

This file records the third-party components that PICO-elevationmap wraps or
depends on, together with their license notices. It is **not** an outbound license
for PICO-elevationmap itself. The project license is still pending an owner
decision; see [docs/open-source-readiness.md](docs/open-source-readiness.md).

When distributing source or binary artifacts, keep these notices with the
distribution and verify the upstream LICENSE files before release.

## MIT-licensed components

### elevation_mapping_cupy_core

Source: https://github.com/leggedrobotics/elevation_mapping_cupy_core

```text
MIT License

Copyright (c) 2022 ETH Zurich, Takahiro Miki

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
```

### elevation_mapping_cupy

Source: https://github.com/leggedrobotics/elevation_mapping_cupy

```text
MIT License

Copyright (c) 2022 ETH Zurich, Takahiro Miki
```

The full MIT license text above (for `elevation_mapping_cupy_core`) applies to
this component as well.

## BSD-3-Clause components

### elevation_mapping

Source: https://github.com/ANYbotics/elevation_mapping

```text
Copyright 2019, ANYbotics AG.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met:

1. Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in
   the documentation and/or other materials provided with the
   distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived
   from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
```

### grid_map

Source: https://github.com/ANYbotics/grid_map

```text
Copyright 2019, ANYbotics AG.
```

The full BSD-3-Clause text above (for `elevation_mapping`) applies to this
component as well.

## Other runtime dependencies

The Python dependencies (`cupy-cuda12x`, `numpy`, `PyYAML`, `rosbags`,
`pyrealsense2`, and transitive packages such as OpenCV) carry their own licenses.
Generate a complete dependency notice before a public release rather than relying
only on this summary.
