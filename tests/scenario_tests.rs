#[cfg(test)]

use nerdbank_algorithms::node_constraint_selection::*;

#[test]
fn nodes_retained() {
    let scenario = Scenario::<&str, bool, 3_usize>::new(&["A", "B", "C"]);
    assert_eq!(["A", "B", "C"], *scenario.nodes);
}

#[test]
fn add_remove_constraint() {
    let mut scenario = Scenario::<&str, bool, 3_usize>::new(&["A", "B", "C"]);
    scenario.add_constraint(Box::new(SelectionCountConstraint {
        nodes: [1, 2],
        min: 1,
        max: 2,
    }));

    assert_eq!(1, scenario.get_constraints().len());
    scenario.remove_constraint(0);
    assert_eq!(0, scenario.get_constraints().len());
}

#[test]
fn create_solution_builder() {
    let builder = SolutionBuilder::new(&["A", "B", "C"], [true, false]);
    assert_eq!(["A", "B", "C"], *builder.scenario.nodes);
    assert_eq!(None, builder[0]);
}

#[test]
fn solution_builder_resolve_partially() {
    let mut builder = SolutionBuilder::new(&["A", "B", "C"], [true, false]);
    builder.scenario.add_constraint(Box::new(SelectionCountConstraint {
        nodes: [0, 1, 2],
        min: 1,
        max: 2,
    }));
    builder.resolve_partially();
    assert_eq!(None, builder[0]);
    assert_eq!(None, builder[1]);
    assert_eq!(None, builder[2]);

    builder.scenario.state.set_node_state(0, true);
    builder.resolve_partially();
    assert_eq!(Some(true), builder[0]);
    assert_eq!(None, builder[1]);
    assert_eq!(None, builder[2]);

    builder.scenario.state.set_node_state(1, true);
    builder.resolve_partially();
    assert_eq!(Some(true), builder[0]);
    assert_eq!(Some(true), builder[1]);
    assert_eq!(Some(false), builder[2]);
}
