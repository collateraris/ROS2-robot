"""Manual hold-to-run GUI; does not automatically arm or hold a heartbeat."""


def main():
    import tkinter as tk
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Bool, String
    from std_srvs.srv import Trigger
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

    rclpy.init()
    node = Node('room_operator_v4')
    deadman = node.create_publisher(Bool, '/room_base/deadman', 1)
    latch = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL)
    estop = node.create_publisher(Bool, '/room_base/estop', latch)
    text = node.create_publisher(String, '/room_nav/text_command', 1)
    arm = node.create_client(Trigger, '/room_base/arm')
    window = tk.Tk()
    window.title('Room robot v4 RF2O — оператор')
    state = {'held': False, 'future': None}
    status = tk.StringVar(value='Удерживай зелёную кнопку, затем ARM. Отпускание останавливает.')
    tk.Label(window, textvariable=status, wraplength=500).pack(padx=12, pady=12)
    held = tk.Button(window, text='УДЕРЖИВАТЬ ДЛЯ ДВИЖЕНИЯ', bg='#b4e7bb', height=3)
    held.pack(fill='x', padx=12)

    def release(event=None):
        state['held'] = False
        deadman.publish(Bool(data=False))

    held.bind('<ButtonPress-1>', lambda event: state.update(held=True))
    # ARM is a keyboard action so pressing it does not require releasing hold.
    window.bind_all('<ButtonRelease-1>', release)
    window.bind('<FocusOut>', release)

    def request_arm(event=None):
        if not state['held']:
            status.set('Сначала удерживай кнопку мышью; ARM — клавиша A.')
        elif not arm.service_is_ready():
            status.set('Сервис room_base/arm недоступен.')
        elif state['future'] is None:
            deadman.publish(Bool(data=True))
            state['future'] = arm.call_async(Trigger.Request())

    tk.Label(window, text='Удерживая кнопку: клавиша A — ARM, Esc — STOP.').pack(pady=8)
    window.bind('<KeyPress-a>', request_arm)
    window.bind('<KeyPress-A>', request_arm)

    def stop(event=None):
        release()
        estop.publish(Bool(data=True))
        text.publish(String(data='стоп'))

    window.bind('<Escape>', stop)
    tk.Button(window, text='STOP', bg='#ef9999', command=stop).pack(fill='x', padx=12)
    entry = tk.Entry(window, width=50)
    entry.insert(0, 'к столу')
    entry.pack(padx=12, pady=12)
    tk.Button(window, text='Отправить задачу',
              command=lambda: text.publish(String(data=entry.get()))).pack(pady=5)
    # Lets the operator submit the pretyped task without releasing hold-to-run.
    window.bind('<Return>', lambda event: text.publish(String(data=entry.get())))
    tk.Label(window, text='Можно набрать команду заранее, удержать кнопку,\n'
                           'Enter — отправить задачу, затем A — запросить ARM.').pack(pady=5)

    def loop():
        rclpy.spin_once(node, timeout_sec=0)
        deadman.publish(Bool(data=state['held']))
        if state['future'] is not None and state['future'].done():
            try:
                response = state['future'].result()
                status.set(('ARM: ' if response.success else 'Отказ: ')+response.message)
            except Exception as exc:
                status.set(str(exc))
            state['future'] = None
        window.after(50, loop)

    def close():
        stop()
        window.destroy()

    window.protocol('WM_DELETE_WINDOW', close)
    window.after(50, loop)
    try:
        window.mainloop()
    finally:
        deadman.publish(Bool(data=False))
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
