# -*- coding: utf-8 -*-
"""Experiment 2: Leader -> Middle(IDM) -> Student."""
import sys
from PyQt6.QtWidgets import QApplication
from three_vehicle_experiment import ThreeVehicleExperimentGUI
from student_controller2 import student_control


class Experiment2GUI(ThreeVehicleExperimentGUI):
    experiment_name = "experiment2"
    window_title = "INTR5100 Experiment 2 - Leader -> IDM -> Student"
    roles = ("leader", "middle", "student")
    idm_front_role = "leader"
    idm_ego_role = "middle"
    student_role = "student"
    student_front_role = "middle"

    def _student_args(self, states):
        return states["leader"], states["middle"], states["student"]


def main():
    app = QApplication(sys.argv)
    app.setApplicationName("INTR5100 Experiment 2")
    w = Experiment2GUI(student_control)
    w.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
