"""Local ROS2 desktop supervisor. Hold-to-run is independent of text planning."""
from __future__ import annotations
import json
import time
import uuid


def window_has_focus(window) -> bool:
    """Only this top-level counts; focus-query errors release the deadman."""
    try:
        focused = window.focus_displayof()
        return focused is not None and focused.winfo_toplevel() == window
    except Exception:
        return False


def main(argv=None):
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Bool
    from .ros_interfaces import message_types,task_to_ros,stop_to_ros
    interfaces=message_types()
    from std_srvs.srv import SetBool
    import tkinter as tk
    from tkinter import ttk
    rclpy.init(args=argv)
    node=Node("roomwm_operator")
    task_pub=node.create_publisher(interfaces.TaskRequest,"/roomwm/task",1)
    stop_pub=node.create_publisher(interfaces.Stop,"/roomwm/emergency_stop",1)
    deadman_pub=node.create_publisher(Bool,"/roomwm/deadman",1)
    arm_client=node.create_client(SetBool,"/roomwm/arm")
    window=tk.Tk(); window.title("Room robot · оператор"); window.geometry("860x600")
    window.configure(padx=18,pady=16)
    held=False
    mouse_held=False
    space_held=False
    release_after=None
    closing=False
    pi_seen=0.
    feedback=tk.StringVar(value="Ожидание Raspberry Pi")
    pc_feedback=tk.StringVar(value="Ожидание восприятия")
    calibration_ack=tk.BooleanVar(value=False)
    text=tk.StringVar(value="подъедь к человеку и обходи препятствия")

    def stopped(reason="operator_stop"):
        nonlocal held,mouse_held,space_held,release_after
        if release_after is not None:
            window.after_cancel(release_after)
            release_after=None
        held=mouse_held=space_held=False
        deadman_pub.publish(Bool(data=False)); stop_pub.publish(stop_to_ros(reason))
        hold_button.configure(text="УДЕРЖИВАТЬ ДЛЯ ДВИЖЕНИЯ")

    def set_goal(kind,**data):
        stopped("operator_new_goal")
        goal={"goal_id":uuid.uuid4().hex,"kind":kind,**data}
        task_pub.publish(task_to_ros(goal))
        feedback.set("Цель отправлена. Подтверди неподвижность, удерживай разрешение и нажми ARM.")

    def arm():
        if not calibration_ack.get():
            feedback.set("ARM: сначала визуально проверь, что робот полностью остановлен.")
            return
        if not held:
            feedback.set("ARM: удерживай пробел или кнопку разрешения движения.")
            return
        if not arm_client.service_is_ready():
            feedback.set("Сервис Raspberry Pi /roomwm/arm не найден.")
            return
        request=SetBool.Request(); request.data=True
        future=arm_client.call_async(request)
        def answer(result):
            try:
                value=result.result(); feedback.set(value.message)
            except Exception as exc:
                feedback.set(str(exc)); stopped("arm_service_failure")
        future.add_done_callback(answer)

    def holding(value,source):
        nonlocal held,mouse_held,space_held
        if source=="mouse": mouse_held=value
        else: space_held=value
        was=held; held=mouse_held or space_held
        if was and not held:
            deadman_pub.publish(Bool(data=False))
        hold_button.configure(text="РАЗРЕШЕНИЕ УДЕРЖИВАЕТСЯ" if held else "УДЕРЖИВАТЬ ДЛЯ ДВИЖЕНИЯ")

    def space_down(event):
        nonlocal release_after
        if not window_has_focus(window) or window.focus_get() is entry:
            return
        if release_after is not None:
            window.after_cancel(release_after); release_after=None
        holding(True,"space")
        return "break"

    def space_up(event):
        nonlocal release_after
        # Linux can synthesize a release/press pair on auto-repeat.
        def released():
            nonlocal release_after
            release_after=None
            holding(False,"space")
        if release_after is not None:
            window.after_cancel(release_after)
        release_after=window.after_idle(released)

    ttk.Label(window,text="Сначала цель → затем удержание → ARM",font=("Sans",16,"bold")).pack(anchor="w",pady=(0,10))
    ttk.Label(window,text="Текст выбирает задачу. Каждое движение — отдельный ограниченный импульс.",wraplength=810).pack(anchor="w")
    entry=ttk.Entry(window,textvariable=text,font=("Sans",13)); entry.pack(fill="x",pady=10)
    ttk.Button(window,text="Отправить текстовую команду",command=lambda:set_goal("text",text=text.get())).pack(anchor="w")
    ttk.Separator(window).pack(fill="x",pady=14)
    ttk.Label(window,text="Сбор данных (PC должен работать с --mode teleop):").pack(anchor="w")
    manual=ttk.Frame(window); manual.pack(anchor="w",pady=8)
    for label,direction in (("↰ Дуга влево","left"),("↑ Вперёд","forward"),("↱ Дуга вправо","right")):
        ttk.Button(manual,text=label,command=lambda d=direction:set_goal("manual",direction=d)).pack(side="left",padx=4)
    ttk.Checkbutton(window,text="Я вижу, что робот полностью остановлен; место первого импульса проверено",variable=calibration_ack).pack(anchor="w",pady=10)
    hold_button=ttk.Button(window,text="УДЕРЖИВАТЬ ДЛЯ ДВИЖЕНИЯ")
    hold_button.pack(fill="x",pady=8)
    hold_button.bind("<ButtonPress-1>",lambda event:holding(True,"mouse"))
    hold_button.bind("<ButtonRelease-1>",lambda event:holding(False,"mouse"))
    hold_button.bind("<Leave>",lambda event:holding(False,"mouse"))
    controls=ttk.Frame(window); controls.pack(fill="x",pady=8)
    ttk.Button(controls,text="ARM · включить после проверки",command=arm).pack(side="left")
    tk.Button(controls,text="СТОП",bg="#b91c1c",fg="white",font=("Sans",16,"bold"),command=stopped).pack(side="right",ipadx=50)
    ttk.Label(window,textvariable=feedback,wraplength=810).pack(anchor="w",pady=10)
    ttk.Label(window,textvariable=pc_feedback,wraplength=810).pack(anchor="w",pady=6)
    ttk.Label(window,text="Пробел — удержание, Escape — стоп. Потеря фокуса отпускает разрешение.",wraplength=810).pack(anchor="w",pady=8)

    def status(msg):
        nonlocal pi_seen
        try:
            if msg.schema_version!=2:
                raise ValueError("Wrong status schema")
            pi_seen=time.monotonic()
            age=f"{msg.frame_age_s:.3f}s" if msg.frame_age_valid else "неизвестен"
            feedback.set(f"Pi: {msg.state} · {msg.reason} · ARM={msg.armed} · MCU={msg.motor_ready} · возраст кадра≤{age}")
        except (ValueError,KeyError,TypeError,AttributeError):
            stopped("bad_pi_status")

    def pc_status(msg):
        try:
            pc_feedback.set(f"PC: {msg.reason} · {msg.inference_ms:.0f} мс · обзор: {msg.current_perception.path_reason}")
        except (ValueError,KeyError,TypeError,AttributeError):
            pass

    node.create_subscription(interfaces.SafetyStatus,"/roomwm/status",status,1)
    node.create_subscription(interfaces.PlannerStatus,"/roomwm/pc_status",pc_status,1)
    window.bind("<KeyPress-space>",space_down)
    window.bind("<KeyRelease-space>",space_up)
    window.bind("<Escape>",lambda event:stopped())
    def check_focus_after_transition():
        # FocusOut can fire before Tk has completed the focus transfer. Checking
        # immediately may still see the old widget and retain a stale hold.
        if not closing and not window_has_focus(window):
            stopped("operator_window_lost_focus")
    window.bind("<FocusOut>",lambda event:window.after_idle(check_focus_after_transition))

    def tick():
        if closing or not rclpy.ok():
            return
        rclpy.spin_once(node,timeout_sec=0.)
        # Independent periodic check also covers a missed focus event.
        if held and not window_has_focus(window):
            stopped("operator_window_lost_focus")
        elif space_held and window.focus_get() is entry:
            holding(False,"space")
        if pi_seen and time.monotonic()-pi_seen>1.:
            stopped("pi_status_lost")
        deadman_pub.publish(Bool(data=held))
        window.after(50,tick)

    def close():
        nonlocal closing
        closing=True
        stopped("operator_window_closed")
        window.destroy()

    window.protocol("WM_DELETE_WINDOW",close)
    tick()
    try:
        window.mainloop()
    finally:
        deadman_pub.publish(Bool(data=False))
        node.destroy_node(); rclpy.shutdown()


if __name__=="__main__":
    main()
