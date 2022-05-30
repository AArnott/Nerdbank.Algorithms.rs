#[cfg(test)]
#[macro_use] extern crate assert_matches;

use nerdbank_algorithms::{self, Constraint, Scenario, SelectionCountConstraint};

#[test]
fn nodes_retained() {
    let scenario = Scenario::<&str, bool, 3_usize>::new(["A", "B", "C"]);
    assert_eq!(["A", "B", "C"], scenario.nodes);
}

#[test]
fn add_constraint() {
    let mut scenario = Scenario::<&str, bool, 3_usize>::new(["A", "B", "C"]);
    scenario.add_constraint(Constraint::SelectionCount(SelectionCountConstraint {
        nodes: vec![1, 2],
        min: 1,
        max: 2,
    }));

    assert_eq!(1, scenario.get_constraints().len());
    assert_matches!(scenario.get_constraints()[0], Constraint::SelectionCount(_));
}
