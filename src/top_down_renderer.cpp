/*
 * Copyright (c) 2019, The Regents of the University of California (Regents).
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *
 *    2. Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *
 *    3. Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Please contact the author(s) of this library if you have any questions.
 * Authors: David Fridovich-Keil   ( dfk@eecs.berkeley.edu )
 */

///////////////////////////////////////////////////////////////////////////////
//
// Core renderer for 2D top-down trajectories. Integrates with DearImGui.
//
///////////////////////////////////////////////////////////////////////////////

#include <ilqgames/gui/top_down_renderer.h>
#include <ilqgames/utils/operating_point.h>
#include <ilqgames/utils/types.h>

#include <imgui/imgui.h>
#include <math.h>
#include <vector>

namespace ilqgames {

float TopDownRenderer::time_ = 0.0;
int TopDownRenderer::iterate_ = 0;

void TopDownRenderer::Render(const std::vector<Dimension>& x_idxs,
                             const std::vector<Dimension>& y_idxs,
                             const std::vector<Dimension>& heading_idxs) const {
  CHECK_EQ(x_idxs.size(), y_idxs.size());
  CHECK_EQ(x_idxs.size(), heading_idxs.size());

  // Do nothing if no iterates yet.
  if (log_->NumIterates() == 0) return;
  const size_t num_agents = x_idxs.size();

  // Make a slider to get the desired interpolation time.
  ImGui::SliderFloat("Interpolation Time (s)", &time_, 0.0, log_->FinalTime());

  // Make a slider to get the desired iterate.
  ImGui::SliderInt("Iterate", &iterate_, 0, log_->NumIterates() - 1);

  // Get the draw list for this window.
  ImDrawList* draw_list = ImGui::GetWindowDrawList();

  // (1) Draw this trajectory iterate.
  const ImU32 trajectory_color = ImColor(ImVec4(1.0, 1.0, 1.0, 0.5));
  const float trajectory_thickness = std::min(1.0, LengthToPixels(0.5));

  ImVec2 points[log_->NumTimeSteps()];
  for (size_t ii = 0; ii < num_agents; ii++) {
    for (size_t kk = 0; kk < log_->NumTimeSteps(); kk++) {
      points[kk] =
          PositionToWindowCoordinates(log_->State(iterate_, kk, x_idxs[ii]),
                                      log_->State(iterate_, kk, y_idxs[ii]));
    }

    draw_list->AddPolyline(&points, log_->NumTimeSteps(), trajectory_color,
                           true, trajectory_thickness);
  }

  // Agent colors will all be greenish. Also specify circle radius and triangle
  // base and height (in pixels).
  const ImU32 agent_color = ImColor(ImVec4(0.0, 0.75, 0.15, 1.0));
  const float agent_radius = std::min(5.0, LengthToPixels(2.5));
  const float agent_base = std::min(6.0, LengthToPixels(3.0));
  const float agent_height = std::min(10.0, LengthToPixels(5.0));

  // Draw each position as either an isosceles triangle (if heading idx is
  // >= 0) or a circle (if heading idx < 0).
  for (size_t ii = 0; ii < num_agents; ii++) {
    const ImVec2 p = PositionToWindowCoordinates(
        log_->InterpolateState(iterate_, time_, x_idxs[ii]),
        log_->InterpolateState(iterate_, time_, y_idxs[ii]));

    if (heading_idxs[ii] < 0)
      draw_list->AddCircleFilled(p, agent_radius, agent_color);
    else {
      const float heading = HeadingToWindowCoordinates(
          log_->InterpolateState(iterate_, time_, heading_idxs[ii]));
      const float cheading = std::cos(heading);
      const float sheading = std::sin(heading);

      // Triangle vertices (top, bottom left, bottom right in Frenet frame).
      // NOTE: this may not be in CCW order. Not sure if that matters.
      const ImVec2 top(p.x + agent_height * cheading,
                       p.y + agent_height * sheading);
      const ImVec2 bl(p.x - 0.5 * agent_base * sheading,
                      p.y + 0.5 * agent_base * cheading);
      const ImVec2 br(p.x + 0.5 * agent_base * sheading,
                      p.y - 0.5 * agent_base * cheading);

      draw_list->AddTriangleFilled(bl, br, top, agent_color)
    }
  }
}

}  // namespace ilqgames