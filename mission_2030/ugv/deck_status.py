class DeckStatus:
    def __init__(self):
        self.is_ready = True

    def set_ready(self, ready: bool):
        self.is_ready = ready

    def verify_deck_clear(self) -> bool:
        """ Returns true if deck is ready for landing """
        return self.is_ready
