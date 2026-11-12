"""Idempotent SLAM startup transitions; never reset an active node."""


class StartupTransitions:
    def __init__(self,limit=3):
        self.limit=limit
        self.attempts={1:0,3:0}

    def next(self,state,pending=False):
        transition={1:1,2:3}.get(state)
        if pending or transition is None or self.attempts[transition]>=self.limit:return None
        self.attempts[transition]+=1
        return transition

    def exhausted(self,state):
        transition={1:1,2:3}.get(state)
        return transition is not None and self.attempts[transition]>=self.limit
