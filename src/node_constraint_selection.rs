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

pub struct Scenario<'nodes, 'constraints, TNode, TNodeState: Copy + Eq, const NODE_COUNT: usize> {
    /// The selection state for each node.
    pub selection_state: [Option<TNodeState>; NODE_COUNT],
    pub nodes: &'nodes [TNode; NODE_COUNT],
    pub constraints: Vec<&'constraints dyn IConstraint<TNodeState>>,
    version: i32,
}

impl<'nodes, 'constraints, TNode, TNodeState: Copy + Eq, const NODE_COUNT: usize> Clone
    for Scenario<'nodes, 'constraints, TNode, TNodeState, NODE_COUNT>
{
    fn clone(&self) -> Self {
        Self {
            selection_state: self.selection_state.clone(),
            nodes: self.nodes,
            constraints: self.constraints.clone(),
            version: self.version,
        }
    }
}

impl<'nodes, 'constraints, TNode, TNodeState: Copy + Eq, const NODE_COUNT: usize>
    Scenario<'nodes, 'constraints, TNode, TNodeState, NODE_COUNT>
{
    pub fn new(
        nodes: &'nodes [TNode; NODE_COUNT],
    ) -> Scenario<'nodes, 'constraints, TNode, TNodeState, NODE_COUNT> {
        Scenario {
            selection_state: [None; NODE_COUNT],
            nodes: nodes,
            constraints: Vec::new(),
            version: 0,
        }
    }

    pub fn get_constraints<'a>(&'a self) -> &Vec<&dyn IConstraint<TNodeState>> {
        &self.constraints
    }

    pub fn add_constraint(&mut self, constraint: &'constraints dyn IConstraint<TNodeState>) {
        self.version += 1;
        self.constraints.push(constraint);
    }

    pub fn remove_constraint(&mut self, index: usize) -> &dyn IConstraint<TNodeState> {
        self.version += 1;
        self.constraints.remove(index)
    }
}

pub trait ScenarioTrait<TNodeState: Copy + Eq> {
    fn get_node_state<'nodes>(&'nodes self, index: usize) -> &'nodes Option<TNodeState>;
    fn set_node_state(&mut self, index: usize, value: TNodeState);
    fn reset_node_state(&mut self, index: usize, value: Option<TNodeState>);
}

impl<'nodes, 'constraints, TNode, TNodeState: Copy + Eq + PartialEq, const NODE_COUNT: usize>
    ScenarioTrait<TNodeState> for Scenario<'nodes, 'constraints, TNode, TNodeState, NODE_COUNT>
{
    fn get_node_state<'a>(&'a self, index: usize) -> &'a Option<TNodeState> {
        &self[index]
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

impl<'nodes, 'constraints, TNode, TNodeState: Copy + Eq, const NODE_COUNT: usize> Index<usize>
    for Scenario<'nodes, 'constraints, TNode, TNodeState, NODE_COUNT>
{
    type Output = Option<TNodeState>;
    fn index<'a>(&'a self, i: usize) -> &'a Option<TNodeState> {
        &self.selection_state[i]
    }
}

/// Describes some known constraint on the final solution
/// and tests whether a partial solution satisfies the constraint.
pub trait IConstraint<TNodeState: Copy + Eq> {
    /// Gets the state of the constraint with respect to a given scenario.
    /// # Returns
    /// A collection of flags that represent the state.
    fn get_state(&self, scenario: &dyn ScenarioTrait<TNodeState>) -> ConstraintStates;
    /// Sets any indeterminate nodes to selected or unselected based on this constraint, if possible.
    /// # Returns
    /// A value indicating whether any indeterminate nodes were changed.
    fn resolve(&self, scenario: &mut dyn ScenarioTrait<TNodeState>) -> bool;
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
    fn get_state(&self, scenario: &dyn ScenarioTrait<bool>) -> ConstraintStates {
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
    fn resolve(&self, scenario: &mut dyn ScenarioTrait<bool>) -> bool {
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
        scenario: &mut dyn ScenarioTrait<bool>,
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

    fn get_node_stats(&self, scenario: &dyn ScenarioTrait<bool>) -> NodeStats {
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
    node: usize,
    value: TNodeState,
}

impl<TNodeState: Copy + Eq + Eq> IConstraint<TNodeState> for SetOneNodeValueConstraint<TNodeState> {
    fn get_state(&self, scenario: &dyn ScenarioTrait<TNodeState>) -> ConstraintStates {
        match scenario.get_node_state(self.node) {
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

    fn resolve(&self, scenario: &mut dyn ScenarioTrait<TNodeState>) -> bool {
        if let None = scenario.get_node_state(self.node) {
            scenario.set_node_state(self.node, self.value);
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
    'constraints,
    TNode,
    TNodeState: Copy + Eq,
    const NODE_COUNT: usize,
    const NODE_STATE_COUNT: usize,
> {
    pub scenario: Scenario<'nodes, 'constraints, TNode, TNodeState, NODE_COUNT>,
    _resolved_states: [TNodeState; NODE_STATE_COUNT],
    full_refresh_needed: bool,
}

impl<
        'nodes,
        'constraints,
        TNode,
        TNodeState: Copy + Eq + Hash,
        const NODE_COUNT: usize,
        const NODE_STATE_COUNT: usize,
    > SolutionBuilder<'nodes, 'constraints, TNode, TNodeState, NODE_COUNT, NODE_STATE_COUNT>
{
    pub fn new(
        nodes: &'nodes [TNode; NODE_COUNT],
        resolved_states: [TNodeState; NODE_STATE_COUNT],
    ) -> SolutionBuilder<'nodes, 'constraints, TNode, TNodeState, NODE_COUNT, NODE_STATE_COUNT>
    {
        SolutionBuilder {
            scenario: Scenario::new(nodes),
            _resolved_states: resolved_states,
            full_refresh_needed: false,
        }
    }

    pub fn resolve_partially(&mut self) {
        let mut candidate = self.scenario.clone();
        if self.full_refresh_needed {
            for i in 0..NODE_COUNT {
                candidate.reset_node_state(i, None);
            }
        }

        Self::resolve_scenario_partially(&mut candidate);
        self.scenario = candidate;
        self.full_refresh_needed = false
    }

    fn resolve_scenario_partially(
        scenario: &mut Scenario<'nodes, 'constraints, TNode, TNodeState, NODE_COUNT>,
    ) {
        // Keep looping through constraints asking each one to resolve nodes until no changes are applied.
        loop {
            let mut any_resolved = false;

            for i in 0..scenario.constraints.len() {
                let constraint = scenario.constraints[i];
                let prior_version = scenario.version;
                let resolved = constraint.resolve(scenario);
                if resolved && prior_version == scenario.version {
                    panic!("Constraint returned true without changing the scenario.");
                }

                any_resolved |= resolved;
            }

            if !any_resolved {
                break;
            }
        }
    }

    fn enumerate_solutions(
        basis: &Scenario<'nodes, 'constraints, TNode, TNodeState, NODE_COUNT>,
        first_node: usize,
        stats: &mut SolutionStats<TNodeState, NODE_COUNT>,
    ) {
        stats.considered_scenarios += 1;
        let mut can_any_constraint_be_broken = false;
        for j in 0..basis.constraints.len() {
            let constraint = basis.constraints[j];
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

        for i in first_node..NODE_COUNT {
            if let Some(_) = basis.get_node_state(i) {
                // Skip any node that already has a set value.
                continue;
            }

            // We don't need to enumerate possibilities for a node for which no constraints exist.

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
    > Index<usize>
    for SolutionBuilder<'nodes, 'constraints, TNode, TNodeState, NODE_COUNT, NODE_STATE_COUNT>
{
    type Output = Option<TNodeState>;
    fn index<'a>(&'a self, i: usize) -> &'a Option<TNodeState> {
        &self.scenario.get_node_state(i)
    }
}

struct SolutionStats<TNodeState: Copy + Eq, const NODE_COUNT: usize> {
    stop_after_first_solution_found: bool,
    solutions_found: u64,
    nodes_resolved_state_in_solutions: [HashMap<TNodeState, u64>; NODE_COUNT],
    considered_scenarios: u64,
}

impl<TNodeState: Copy + Eq + Hash, const NODE_COUNT: usize> SolutionStats<TNodeState, NODE_COUNT> {
    fn record_solution_found(&mut self, scenario: &dyn ScenarioTrait<TNodeState>) {
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
