#  Copyright (c) 2024 Feng Yang
#
#  I am making my contributions/submissions to this project solely in my
#  personal capacity and am not conveying any rights to any intellectual
#  property of any third parties.

import os

import mujoco
import mujoco.viewer
import numpy as np
from pyPlan import ompl

if __name__ == '__main__':
    path = os.path.join(os.path.dirname(__file__),
                        '../../mujoco_menagerie/franka_emika_panda/scene.xml')

    spec = mujoco.MjSpec()
    spec.from_file(path)
    body = spec.worldbody.add_body()
    body.name = "block"
    body.pos = [0, .6, .4]
    body.quat = [.5, -.2, 1, 0]
    geom = body.add_geom()
    geom.rgba = [1, 0, 0, 1]
    geom.type = mujoco.mjtGeom.mjGEOM_BOX
    geom.size = [.15, .15, .15]

    model = spec.compile()
    data = mujoco.MjData(model)

    state_space = ompl.CompoundStateSpace()
    for i in range(model.nu):
        if model.actuator_ctrllimited[i]:
            subspace = ompl.RealVectorStateSpace(1)
            subspace.setBounds(model.actuator_ctrlrange[i][0], model.actuator_ctrlrange[i][1])
            state_space.addSubspace(subspace, 1)

    space_information = ompl.SpaceInformation(state_space)
    simple_setup = ompl.geometry.SimpleSetup(space_information)


    def is_valid(state: ompl.State) -> bool:
        compound = state.asCompoundState()
        ctrl = compound.toVector(state_space)

        mujoco.mj_resetData(model, data)
        data.ctrl = ctrl
        mujoco.mj_fwdPosition(model, data)

        return len(data.contact) == 2


    checker = ompl.CustomStateValidityChecker(space_information, is_valid)
    simple_setup.setStateValidityChecker(checker)

    start = ompl.ScopedState(state_space)
    start.set([-2.89, -1.52, -1.59, -1.24, -1.19, 2.49, 0.753, 0])

    goals = ompl.GoalStates(space_information)
    for i in range(1):
        goal = ompl.ScopedState(state_space)
        goal.set([2.4, 1.34, -0.811, -1.17, 0, 0.604, 0, 0])
        goals.addState(goal)

    simple_setup.clear()
    simple_setup.setStartState(start)
    simple_setup.setGoal(goals)

    planner = ompl.geometry.RRTConnect(space_information)
    simple_setup.setPlanner(planner)
    simple_setup.setup()
    solved = simple_setup.solve()
    paths = []
    if solved:
        path = simple_setup.getSolutionPath()
        for i in range(path.getStateCount()):
            compound = path.getState(i).asCompoundState()
            ctrl = compound.toVector(state_space)
            paths.append(np.array(ctrl))
        print(paths)

    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.distance = 1.7
        viewer.cam.elevation = -15
        viewer.cam.azimuth = -130
        viewer.cam.lookat = (0, 0, .3)

        with viewer.lock():
            viewer.opt.frame = mujoco.mjtFrame.mjFRAME_CONTACT
            viewer.opt.label = mujoco.mjtLabel.mjLABEL_CONTACTPOINT

        path_id = 0
        substeps_id = 0
        substeps = 0
        dt = 0.01
        mujoco.mj_resetData(model, data)

        while viewer.is_running():
            if path_id < len(paths) - 1 and solved:
                if substeps == 0:
                    distance = np.linalg.norm(paths[path_id + 1] - paths[path_id])
                    substeps = int(distance / dt)
                    substeps_id = 0

                percent = float(substeps_id) / float(substeps)
                internal_path = (1 - percent) * paths[path_id] + percent * paths[path_id + 1]
                data.ctrl = internal_path
                substeps_id += 1

                if substeps_id > substeps:
                    substeps = 0
                    path_id += 1
            else:
                path_id = 0
                mujoco.mj_resetData(model, data)

            # mj_step can be replaced with code that also evaluates
            # a policy and applies a control signal before stepping the physics.
            mujoco.mj_step(model, data)

            # Pick up changes to the physics state, apply perturbations, update options from GUI.
            viewer.sync()
