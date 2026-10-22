"""One operator click releases one spawned box for observe/pick/place/return."""


class ManualPickGate:
    def __init__(self, total):
        self.total = total
        self.number = 0
        self.box_id = ""
        self.stage = "STARTING"
        self.waiting = False
        self.request_id = None
        self.confirmations = 0

    def on_stage(self, stage, box_id):
        self.stage = stage
        if stage == "SPAWN" and box_id != self.box_id:
            self.number += 1
            self.box_id = box_id
            self.request_id = f"{self.number}:{box_id}"
            self.waiting = True

    def start_pick(self, request_id):
        if not self.waiting or request_id != self.request_id:
            return False
        self.waiting = False
        self.confirmations += 1
        return True

    def render(self, imgui):
        imgui.set_next_window_pos(imgui.ImVec2(890, 30), imgui.Cond_.first_use_ever)
        imgui.set_next_window_size(imgui.ImVec2(350, 220), imgui.Cond_.first_use_ever)
        if imgui.begin("Loading control"):
            imgui.text(f"Box {self.number}/{self.total}   {self.box_id}")
            imgui.text("WAITING FOR PICK" if self.waiting else self.stage)
            imgui.separator()
            message = ("Box placed? Click Start Pick." if self.waiting else
                       "All boxes complete." if self.stage == "COMPLETE" else "Current cycle in progress.")
            imgui.text(message)
            imgui.begin_disabled(not self.waiting)
            if imgui.button("Start Pick", imgui.ImVec2(290, 48)):
                self.start_pick(self.request_id)
            imgui.end_disabled()
            imgui.text("One click = observe, pick, place, return.")
        imgui.end()
