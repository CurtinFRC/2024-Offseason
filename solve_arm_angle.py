#!/usr/bin/env python3

from jormungandr.optimization import OptimizationProblem

def main():
    # Find the x, y pair with the largest product for which x + 3y = 36
    problem = OptimizationProblem()

    arm_theta = problem.decision_variable()
    theta = problem.decision_variable()

    y = 5
    x = 3.14159
    h = 1

    problem.minimize(arm_theta)
    problem.subject_to(theta == (atan(y - a * sin(arm_theta)) + h / 2) / x - a * cos(arm_theta))
    problem.solve()

    # x = 18.0, y = 6.0
    print(f"arm_theta = {arm_theta.value()}, theta = {theta.value()}")


if __name__ == "__main__":
    main()
