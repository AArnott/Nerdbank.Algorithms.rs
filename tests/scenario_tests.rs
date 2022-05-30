#[cfg(test)]

use nerdbank_algorithms::{Scenario, SelectionCountConstraint };

#[test]
fn nodes_retained() {
    let scenario = Scenario::<&str, bool, 3_usize>::new(["A", "B", "C"]);
    assert_eq!(["A", "B", "C"], scenario.nodes);
}

#[test]
fn add_remove_constraint() {
    let mut scenario = Scenario::<&str, bool, 3_usize>::new(["A", "B", "C"]);
    scenario.add_constraint(Box::new(SelectionCountConstraint {
        nodes: [1, 2],
        min: 1,
        max: 2,
    }));

    assert_eq!(1, scenario.get_constraints().len());
    scenario.remove_constraint(0);
    assert_eq!(0, scenario.get_constraints().len());
}
