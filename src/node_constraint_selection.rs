use std::{collections::HashMap, hash::Hash, ops::Index};

bitflags! {
    /// Describes the present state of a constraint.
    pub struct ConstraintStates: u8 {
        /// The lack of any other flags.
        const NONE = 0x0;
        /// Indicates that the constraint is satisfied given the current node state or may be satisfied if certain indeterminate nodes resolve to particular states.
        const SATISFIABLE = 0x1;
        /// Indicates that the constraint is satisfied given the nodes that are already selected or unselected.
        const SATISFIED = Self::SATISFIABLE.bits | 0x2;
        /// Indicates that the constraint can set one or more indeterminate nodes as selected or unselected.
        const RESOLVABLE = 0x4;
        /// Indicates that none nodes in the constraint are in an indeterminate state.
        const RESOLVED = 0x8;
        /// Indicates that changes to indeterminate nodes could render the constraint broken.
        const BREAKABLE = 0x10;
    }
}

#[derive(Clone)]
pub struct Experiment<TNodeState: Copy + Eq, const NODE_COUNT: usize> {
    selection_state: [Option<TNodeState>; NODE_COUNT],
    version: i32,
}

pub trait SelectionStateTrait<TNodeState: Copy + Eq> {
    fn get_node_state<'a>(&'a self, index: usize) -> &'a Option<TNodeState>;
    fn set_node_state(&mut self, index: usize, value: TNodeState);
    fn reset_node_state(&mut self, index: usize, value: Option<TNodeState>);
}

impl<TNodeState: Copy + Eq, const NODE_COUNT: usize> Experiment<TNodeState, NODE_COUNT> {
    fn new() -> Self {
        Experiment {
            version: 0,
            selection_state: [None; NODE_COUNT],
        }
    }
}

impl<TNodeState: Copy + Eq, const NODE_COUNT: usize> SelectionStateTrait<TNodeState>
    for Experiment<TNodeState, NODE_COUNT>
{
    fn get_node_state<'a>(&'a self, index: usize) -> &'a Option<TNodeState> {
        &self.selection_state[index]
    }

    fn set_node_state(&mut self, index: usize, value: TNodeState) {
        if let None = self.selection_state[index] {
            self.reset_node_state(index, Some(value));
        } else {
            panic!("Node is already set.");
        }
    }

    fn reset_node_state(&mut self, index: usize, value: Option<TNodeState>) {
        if !self.selection_state[index].eq(&value) {
            self.selection_state[index] = value;
            self.version += 1;
        }
    }
}

pub struct Scenario<'nodes, TNode, TNodeState: Copy + Eq, const NODE_COUNT: usize> {
    /// The selection state for each node.
    pub state: Experiment<TNodeState, NODE_COUNT>,
    pub nodes: &'nodes [TNode; NODE_COUNT],
    pub constraints: Vec<Box<dyn IConstraint<TNodeState>>>,
}

impl<'nodes, TNode, TNodeState: Copy + Eq, const NODE_COUNT: usize>
    Scenario<'nodes, TNode, TNodeState, NODE_COUNT>
{
    pub fn new(
        nodes: &'nodes [TNode; NODE_COUNT],
    ) -> Scenario<'nodes, TNode, TNodeState, NODE_COUNT> {
        Scenario {
            state: Experiment::new(),
            nodes: nodes,
            constraints: Vec::new(),
        }
    }

    pub fn get_constraints<'a>(&'a self) -> &Vec<Box<dyn IConstraint<TNodeState>>> {
        &self.constraints
    }

    pub fn add_constraint(&mut self, constraint: Box<dyn IConstraint<TNodeState>>) {
        self.constraints.push(constraint);
    }

    pub fn remove_constraint(&mut self, index: usize) -> Box<dyn IConstraint<TNodeState>> {
        self.constraints.remove(index)
    }
}

impl<'nodes, TNode, TNodeState: Copy + Eq, const NODE_COUNT: usize> Index<usize>
    for Scenario<'nodes, TNode, TNodeState, NODE_COUNT>
{
    type Output = Option<TNodeState>;
    fn index<'a>(&'a self, i: usize) -> &'a Option<TNodeState> {
        &self.state.get_node_state(i)
    }
}

/// Describes some known constraint on the final solution
/// and tests whether a partial solution satisfies the constraint.
pub trait IConstraint<TNodeState: Copy + Eq> {
    /// Gets the nodes that the constraint applies to.
    fn nodes(&self) -> &[usize];
    /// Gets the state of the constraint with respect to a given scenario.
    /// # Returns
    /// A collection of flags that represent the state.
    fn get_state(&self, scenario: &dyn SelectionStateTrait<TNodeState>) -> ConstraintStates;
    /// Sets any indeterminate nodes to selected or unselected based on this constraint, if possible.
    /// # Returns
    /// A value indicating whether any indeterminate nodes were changed.
    fn resolve(&self, scenario: &mut dyn SelectionStateTrait<TNodeState>) -> bool;
}

#[derive(Debug)]
pub struct SelectionCountConstraint<const NODE_COUNT: usize> {
    /// An array of indexes into scenario nodes that are included in this constraint.
    pub nodes: [usize; NODE_COUNT],
    /// The minimum allowable nodes that may be selected.
    pub min: usize,
    /// The maximum allowable nodes that may be selected.
    pub max: usize,
}

impl<const NODE_COUNT: usize> IConstraint<bool> for SelectionCountConstraint<NODE_COUNT> {
    fn nodes(&self) -> &[usize] {
        &self.nodes
    }

    fn get_state(&self, scenario: &dyn SelectionStateTrait<bool>) -> ConstraintStates {
        let stats = self.get_node_stats(scenario);
        let mut states = ConstraintStates::NONE;

        if self.is_satisfiable(&stats) {
            states |= ConstraintStates::SATISFIABLE;
            if self.is_satisfied(&stats) {
                states |= ConstraintStates::SATISFIED;
            }
        }

        if stats.is_resolved() {
            states |= ConstraintStates::RESOLVED;
        } else {
            if self.can_resolve_by_selecting(&stats) || self.can_resolve_by_unselecting(&stats) {
                states |= ConstraintStates::RESOLVABLE;
            }

            if self.is_breakable(&stats) {
                states |= ConstraintStates::BREAKABLE;
            }
        }

        states
    }
    fn resolve(&self, scenario: &mut dyn SelectionStateTrait<bool>) -> bool {
        let stats = self.get_node_stats(scenario);
        if self.can_resolve_by_unselecting(&stats) {
            // If the maximum nodes have already been selected, unselect the rest.
            self.mark_indeterminate_nodes(scenario, false)
        } else if self.can_resolve_by_selecting(&stats) {
            // If so many nodes have been UNselected that the remaining nodes equal the minimum allowed selected nodes, select the rest.
            self.mark_indeterminate_nodes(scenario, true)
        } else {
            // We cannot resolve yet.
            false
        }
    }
}

impl<const NODE_COUNT: usize> SelectionCountConstraint<NODE_COUNT> {
    fn is_satisfiable(&self, stats: &NodeStats) -> bool {
        self.min <= stats.selected + stats.indeterminate && self.max >= stats.selected
    }

    fn is_satisfied(&self, stats: &NodeStats) -> bool {
        self.min <= stats.selected && self.max >= stats.selected
    }

    fn can_resolve_by_selecting(&self, stats: &NodeStats) -> bool {
        stats.unselected == self.nodes.len() - self.min
    }

    fn can_resolve_by_unselecting(&self, stats: &NodeStats) -> bool {
        stats.selected == self.max
    }

    /// Gets a value indicating whether this constraint may still be broken in the future.
    fn is_breakable(&self, stats: &NodeStats) -> bool {
        // it is already broken...
        !self.is_satisfiable(stats) ||
        // or there are enough indeterminate nodes to not count toward the minimum...
        stats.selected < self.min ||
        // or the number of selected nodes may yet exceed the maximum...
        stats.selected + stats.indeterminate > self.max
    }

    fn mark_indeterminate_nodes(
        &self,
        scenario: &mut dyn SelectionStateTrait<bool>,
        select: bool,
    ) -> bool {
        let mut changed = false;
        for node in self.nodes.iter() {
            if let None = scenario.get_node_state(*node) {
                scenario.set_node_state(*node, select);
                changed = true;
            }
        }

        changed
    }

    fn get_node_stats(&self, scenario: &dyn SelectionStateTrait<bool>) -> NodeStats {
        let mut selected = 0;
        let mut unselected = 0;
        let mut indeterminate = 0;
        for node in self.nodes.iter() {
            match scenario.get_node_state(*node) {
                Some(true) => {
                    selected += 1;
                }
                Some(false) => {
                    unselected += 1;
                }
                None => {
                    indeterminate += 1;
                }
            }
        }

        NodeStats {
            selected,
            unselected,
            indeterminate,
        }
    }
}

#[derive(Debug)]
pub struct SetOneNodeValueConstraint<TNodeState: Copy + Eq> {
    node: [usize; 1],
    value: TNodeState,
}

impl<TNodeState: Copy + Eq + Eq> IConstraint<TNodeState> for SetOneNodeValueConstraint<TNodeState> {
    fn nodes(&self) -> &[usize] {
        &self.node
    }

    fn get_state(&self, scenario: &dyn SelectionStateTrait<TNodeState>) -> ConstraintStates {
        match scenario.get_node_state(self.node[0]) {
            None => {
                ConstraintStates::RESOLVABLE
                    | ConstraintStates::BREAKABLE
                    | ConstraintStates::SATISFIABLE
            }
            Some(v) => {
                if self.value == *v {
                    ConstraintStates::SATISFIED | ConstraintStates::RESOLVED
                } else {
                    ConstraintStates::RESOLVED
                }
            }
        }
    }

    fn resolve(&self, scenario: &mut dyn SelectionStateTrait<TNodeState>) -> bool {
        if let None = scenario.get_node_state(self.node[0]) {
            scenario.set_node_state(self.node[0], self.value);
            true
        } else {
            false
        }
    }
}

struct NodeStats {
    selected: usize,
    unselected: usize,
    indeterminate: usize,
}

impl NodeStats {
    fn is_resolved(&self) -> bool {
        self.indeterminate == 0
    }
}

pub struct SolutionBuilder<
    'nodes,
    TNode,
    TNodeState: Copy + Eq,
    const NODE_COUNT: usize,
    const NODE_STATE_COUNT: usize,
> {
    pub scenario: Scenario<'nodes, TNode, TNodeState, NODE_COUNT>,
    resolved_states: [TNodeState; NODE_STATE_COUNT],
    full_refresh_needed: bool,
}

impl<
        'nodes,
        TNode,
        TNodeState: Copy + Eq + Hash,
        const NODE_COUNT: usize,
        const NODE_STATE_COUNT: usize,
    > SolutionBuilder<'nodes, TNode, TNodeState, NODE_COUNT, NODE_STATE_COUNT>
{
    pub fn new(
        nodes: &'nodes [TNode; NODE_COUNT],
        resolved_states: [TNodeState; NODE_STATE_COUNT],
    ) -> SolutionBuilder<'nodes, TNode, TNodeState, NODE_COUNT, NODE_STATE_COUNT> {
        SolutionBuilder {
            scenario: Scenario::new(nodes),
            resolved_states: resolved_states,
            full_refresh_needed: false,
        }
    }

    pub fn resolve_partially(&mut self) {
        let mut experiment = self.scenario.state.clone();
        if self.full_refresh_needed {
            for i in 0..NODE_COUNT {
                experiment.reset_node_state(i, None);
            }
        }

        Self::resolve_scenario_partially(&self.scenario, &mut experiment);
        self.scenario.state = experiment;
        self.full_refresh_needed = false
    }

    fn resolve_scenario_partially(
        scenario: &Scenario<'nodes, TNode, TNodeState, NODE_COUNT>,
        experiment: &mut Experiment<TNodeState, NODE_COUNT>,
    ) {
        // Keep looping through constraints asking each one to resolve nodes until no changes are applied.
        loop {
            let mut any_resolved = false;

            for i in 0..scenario.constraints.len() {
                let constraint = &scenario.constraints[i];
                let prior_version = experiment.version;
                let resolved = constraint.resolve(experiment);
                if resolved && prior_version == experiment.version {
                    panic!("Constraint returned true without changing the scenario.");
                }

                any_resolved |= resolved;
            }

            if !any_resolved {
                break;
            }
        }
    }

    pub fn analyze_solutions(&self) -> SolutionAnalysis<TNodeState, NODE_COUNT> {
        let mut experiment = self.scenario.state.clone();
        Self::resolve_scenario_partially(&self.scenario, &mut experiment);
        let mut stats = SolutionStats::<TNodeState, NODE_COUNT>::new();
        self.enumerate_solutions(&self.scenario, &experiment, 0, &mut stats);
        SolutionAnalysis {
            viable_solutions_found: stats.solutions_found,
            node_value_count: stats.nodes_resolved_state_in_solutions,
        }
    }

    fn get_constraints_by_node<'a>(
        scenario: &'a Scenario<'nodes, TNode, TNodeState, NODE_COUNT>,
    ) -> Vec<Vec<&'a Box<dyn IConstraint<TNodeState>>>> {
        let mut constraints_by_node: Vec<Vec<&Box<dyn IConstraint<TNodeState>>>> =
            Vec::with_capacity(NODE_COUNT);
        for _ in 0..NODE_COUNT {
            constraints_by_node.push(Vec::new())
        }

        for constraint in scenario.constraints.iter() {
            for node_index in constraint.nodes() {
                let vec = &mut constraints_by_node[*node_index];
                vec.push(constraint);
            }
        }

        constraints_by_node
    }

    fn enumerate_solutions(
        &self,
        scenario: &Scenario<'nodes, TNode, TNodeState, NODE_COUNT>,
        basis: &Experiment<TNodeState, NODE_COUNT>,
        first_node: usize,
        stats: &mut SolutionStats<TNodeState, NODE_COUNT>,
    ) {
        stats.considered_scenarios += 1;
        let mut can_any_constraint_be_broken = false;
        for j in 0..scenario.constraints.len() {
            let constraint = &scenario.constraints[j];
            let state = constraint.get_state(basis);
            if !state.intersects(ConstraintStates::SATISFIABLE) {
                return;
            }

            can_any_constraint_be_broken |= state.intersects(ConstraintStates::BREAKABLE);
        }

        if stats.stop_after_first_solution_found && !can_any_constraint_be_broken {
            // There's nothing we can simulate that would break constraints, so everything we might try constitutes a valid solution.
            // Don't waste time enumerating them.
            stats.record_solution_found(basis);
            return;
        }

        let constraints_by_node = Self::get_constraints_by_node(scenario);

        let mut i = first_node;
        while i < NODE_COUNT {
            if let Some(_) = basis.get_node_state(i) {
                // Skip any node that already has a set value.
                i += 1;
                continue;
            }

            // We don't need to enumerate possibilities for a node for which no constraints exist.
            let applicable_constraints = &constraints_by_node[i];
            if applicable_constraints.is_empty() {
                // Skip any node that can be any value without impact to constraints.
                i += 1;
                continue;
            }

            // Try selecting the node. In doing so, resolve whatever nodes we can immediately.
            for value in self.resolved_states {
                let mut experiment = basis.clone();
                experiment.set_node_state(i, value);
                self.resolve_by_cascading_constraints(
                    scenario,
                    &mut experiment,
                    applicable_constraints,
                );
                self.enumerate_solutions(scenario, &experiment, i + 1, stats);

                if stats.stop_after_first_solution_found && stats.solutions_found > 0 {
                    return;
                }
            }

            // Once we drill into one node, we don't want to drill into any more nodes since
            // we did that via our recursive call.
            break;
        }

        if i >= NODE_COUNT {
            stats.record_solution_found(basis);
        }
    }

    fn resolve_by_cascading_constraints(
        &self,
        scenario: &Scenario<'nodes, TNode, TNodeState, NODE_COUNT>,
        experiment: &mut Experiment<TNodeState, NODE_COUNT>,
        applicable_constraints: &[&Box<dyn IConstraint<TNodeState>>],
    ) {
        let mut any_resolved = false;
        for constraint in applicable_constraints.iter() {
            any_resolved |= constraint.resolve(experiment);
        }

        // If any nodes changed, engage a regular resolve operation
        if any_resolved {
            Self::resolve_scenario_partially(scenario, experiment);
        }
    }
}

impl<
        'nodes,
        'constraints,
        TNode,
        TNodeState: Copy + Eq,
        const NODE_COUNT: usize,
        const NODE_STATE_COUNT: usize,
    > Index<usize> for SolutionBuilder<'nodes, TNode, TNodeState, NODE_COUNT, NODE_STATE_COUNT>
{
    type Output = Option<TNodeState>;
    fn index<'a>(&'a self, i: usize) -> &'a Option<TNodeState> {
        &self.scenario.state.get_node_state(i)
    }
}

struct SolutionStats<TNodeState: Copy + Eq, const NODE_COUNT: usize> {
    stop_after_first_solution_found: bool,
    solutions_found: u64,
    nodes_resolved_state_in_solutions: Vec<HashMap<TNodeState, u64>>,
    considered_scenarios: u64,
}

impl<TNodeState: Copy + Eq + Hash, const NODE_COUNT: usize> SolutionStats<TNodeState, NODE_COUNT> {
    fn new() -> Self {
        let mut vector: Vec<HashMap<TNodeState, u64>> = Vec::with_capacity(NODE_COUNT);
        for _ in 0..NODE_COUNT {
            vector.push(HashMap::new());
        }

        SolutionStats {
            considered_scenarios: 0,
            solutions_found: 0,
            stop_after_first_solution_found: false,
            nodes_resolved_state_in_solutions: vector,
        }
    }

    fn record_solution_found(&mut self, scenario: &dyn SelectionStateTrait<TNodeState>) {
        self.solutions_found += 1;
        if !self.stop_after_first_solution_found {
            for i in 0..NODE_COUNT {
                if let Some(resolved_state) = scenario.get_node_state(i) {
                    let count = self.nodes_resolved_state_in_solutions[i]
                        .entry(*resolved_state)
                        .or_insert(0);
                    *count += 1;
                } else {
                    // This node is not constrained by anything. So it is a free radical and shouldn't be counted as selected or unselected
                    // since solutions are not enumerated based on flipping this.
                }
            }
        }
    }
}

pub struct SolutionAnalysis<TNodeState: Hash, const NODE_COUNT: usize> {
    pub viable_solutions_found: u64,
    pub node_value_count: Vec<HashMap<TNodeState, u64>>,
}
