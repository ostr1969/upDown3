/* -------------------------------------------------------------------------- *
 * OpenSim Moco: RegisterTypes_osimMocoJumpGoal.cpp                   *
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
#include "MocoJumpGoal.h"
#include "RegisterTypes_osimMocoJumpGoal.h"

using namespace OpenSim;

static osimMocoJumpGoalInstantiator instantiator;

OSIMMOCOJUMPGOAL_API void RegisterTypes_osimMocoJumpGoal() {
    try {
        Object::registerType(MocoJumpGoal());
    } catch (const std::exception& e) {
        std::cerr << "ERROR during osimMocoJumpGoal "
                     "Object registration:\n"
                  << e.what() << std::endl;
    }
}

osimMocoJumpGoalInstantiator::osimMocoJumpGoalInstantiator() {
    registerDllClasses();
}

void osimMocoJumpGoalInstantiator::registerDllClasses() {
    RegisterTypes_osimMocoJumpGoal();
}
