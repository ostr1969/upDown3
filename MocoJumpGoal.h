#ifndef MOCO_MOCOJUMPGOAL_H
#define MOCO_MOCOJUMPGOAL_H
/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoJumpGoal.h                                       *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2019 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Christopher Dembia                                              *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0          *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */


#include <Moco/osimMoco.h>
#include "osimMocoJumpGoalDLL.h"

namespace OpenSim {

class OSIMMOCOJUMPGOAL_API MocoJumpGoal : public MocoGoal {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoJumpGoal, MocoGoal);

public:
    MocoJumpGoal() {}
    MocoJumpGoal(std::string name) : MocoGoal(std::move(name)) {}
    MocoJumpGoal(std::string name, double weight)
            : MocoGoal(std::move(name), weight) {}

protected:
    Mode getDefaultModeImpl() const override { return Mode::Cost; }
    bool getSupportsEndpointConstraintImpl() const override { return true; }
    void initializeOnModelImpl(const Model&) const override;
    void calcIntegrandImpl(
            const IntegrandInput& input, double& integrand) const override;
    void calcGoalImpl(
            const GoalInput& input, SimTK::Vector& cost) const override;
};

} // namespace OpenSim
struct GoalInput {
        const SimTK::State& initial_state;
        const SimTK::State& final_state;
        /// This is computed by integrating calcIntegrand().
        const double& integral;
        const double& maxf;
    };


#endif // MOCO_MOCOJUMPGOAL_H
