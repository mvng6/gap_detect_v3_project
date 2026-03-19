import sys
import asyncio

from woosh.proto.robot.robot_pb2 import (
    PoseSpeed,
    TaskProc,
    OperationState,
    ScannerData,
)
from woosh.proto.robot.robot_pack_pb2 import (
    ExecTask,
    ActionOrder,
    Twist,
)
from woosh.proto.util.action_pb2 import kCancel as kActionCancel
from woosh.proto.util.task_pb2 import State as TaskState, Type as TaskType
from woosh.proto.ros.ros_pack_pb2 import (
    CallAction,
    Feedbacks,
)
from woosh.proto.ros.action_pb2 import (
    StepControl,
    ControlAction,
)

from woosh_interface import CommuSettings, NO_PRINT
from woosh_robot import WooshRobot


def print_pose_speed(info: PoseSpeed):
    """打印位姿速度信息"""
    print(f"位姿速度更新:\n{info}")
    print(f"所在地图ID: {info.map_id}")
    print(f"线速度: {info.twist.linear}")
    print(f"角速度: {info.twist.angular}")
    print(f"累计里程: {info.mileage}")
    print(f"坐标 x: {info.pose.x}")
    print(f"坐标 y: {info.pose.y}")
    print(f"坐标 theta: {info.pose.theta}")


def print_task_proc(info: TaskProc):
    """打印任务进度信息"""
    print(f"任务进度信息更新:\n{info}")
    if info.state == TaskState.kCompleted:
        print(f"任务ID: {info.robot_task_id}, 任务完成")


def print_operation_state(state: OperationState):
    """打印运行状态信息"""
    print(f"运行状态更新:\n{state}")
    if state.nav & OperationState.NavBit.kImpede:
        print("机器人遇到障碍物")
    if state.robot & OperationState.RobotBit.kTaskable:
        print("机器人可接受任务状态")
    else:
        print("机器人不可接受任务状态")


def print_feedbacks(fbs: Feedbacks):
    """打印ROS反馈信息"""
    print(f"ros feedbacks:\n{fbs}")
    for fb in fbs.fbs:
        if fb.state == TaskState.kRosSuccess:
            print(f"ros action [{fb.action}] 完成")


def print_scanner_data(data: ScannerData):
    """打印雷达数据"""
    print(f"雷达数据更新:\n{data}")


async def main():
    # 参数处理
    addr = "172.20.254.63"
    port = 10003

    if len(sys.argv) >= 3:
        addr = sys.argv[1]
        port = int(sys.argv[2])

    print(f"连接地址: {addr}:{port}")

    # 初始化SDK
    settings = CommuSettings(
        addr=addr,
        port=port,
        identity="woosdk-demo",
    )

    # 创建机器人实例
    robot = WooshRobot(settings)
    await robot.run()

    # 订阅机器人位姿
    await robot.robot_pose_speed_sub(print_pose_speed, NO_PRINT)

    # 订阅机器人任务信息
    await robot.robot_task_process_sub(print_task_proc, NO_PRINT)

    # 订阅机器人运行状态
    await robot.robot_operation_state_sub(print_operation_state, NO_PRINT)

    # 订阅ros feedbacks
    await robot.feedbacks_sub(print_feedbacks, NO_PRINT)

    # 订阅雷达点云数据
    await robot.scanner_data_sub(print_scanner_data, NO_PRINT)

    # 请求机器人位姿
    pose_speed, ok, msg = await robot.robot_pose_speed_req(
        PoseSpeed(), NO_PRINT, NO_PRINT
    )
    if ok:
        print(f"位姿速度请求成功:\n{pose_speed}")
    else:
        print(f"位姿速度请求失败, msg: {msg}")

    # 请求机器人运行状态
    state, ok, msg = await robot.robot_operation_state_req(
        OperationState(), NO_PRINT, NO_PRINT
    )
    if ok:
        print(f"运行状态请求成功:\n{state}")
        if state.nav & OperationState.NavBit.kImpede:
            print("机器人遇到障碍物")
        if state.robot & OperationState.RobotBit.kTaskable:
            print("机器人可接受任务状态")
        else:
            print("机器人不可接受任务状态")
    else:
        print(f"运行状态请求失败, msg: {msg}")

    # 请求机器人指定任务状态
    task_proc = TaskProc(robot_task_id=66666)
    task_info, ok, msg = await robot.robot_task_process_req(
        task_proc, NO_PRINT, NO_PRINT
    )
    if ok:
        print(f"请求机器人任务状态成功:\n{task_info}")
        if task_info.state == TaskState.kCompleted:
            print(f"任务ID: {task_info.robot_task_id}, 任务完成")
    else:
        print(f"请求机器人任务状态失败, msg: {msg}")

    input("输入回车执行导航\n")

    # 任务请求
    exec_task = ExecTask(
        task_id=66666,
        type=TaskType.kPick,
    )
    exec_task.pose.x = 1.5
    exec_task.pose.y = 0.5
    exec_task.pose.theta = 1.57

    task_result, ok, msg = await robot.exec_task_req(exec_task, NO_PRINT, NO_PRINT)
    if ok:
        print("执行任务请求成功")
    else:
        print(f"执行任务请求失败, msg: {msg}")

    await asyncio.sleep(5)
    input("输入回车取消任务\n")

    # 任务指令
    action_order = ActionOrder(order=kActionCancel)  # 取消任务
    order_result, ok, msg = await robot.action_order_req(
        action_order, NO_PRINT, NO_PRINT
    )
    if ok:
        print("动作指令请求成功")
    else:
        print(f"动作指令请求失败, msg: {msg}")

    await asyncio.sleep(1)
    input("输入回车执行步进\n")

    # 步进请求
    step_control = StepControl()
    step = step_control.steps.add()
    # 直行
    step.mode = StepControl.Step.Mode.kStraight
    step.value = 0.5
    step.speed = 0.25
    step_control.action = ControlAction.kExecute

    call_action = CallAction(step_control=step_control)
    action_result, ok, msg = await robot.call_action_req(
        call_action, NO_PRINT, NO_PRINT
    )
    if ok:
        print("步进控制请求成功")
    else:
        print(f"步进控制请求失败, msg: {msg}")

    await asyncio.sleep(5)
    input("输入回车外任意字符开始遥控\n")

    # 速度控制
    hertz = 20  # 控制周期 hz
    delay = 0.1  # 控制延时 s
    linear = 0.0  # 线速度
    angular = 0.785  # 角速度

    twist = Twist(linear=linear, angular=angular)

    for _ in range(20):
        twist_result, ok, msg = await robot.twist_req(twist, NO_PRINT, NO_PRINT)
        if ok:
            print("速度控制请求成功")
        else:
            print(f"速度控制请求失败, msg: {msg}")

        await asyncio.sleep(delay)

    # 平滑减速
    zero_time = 1.5  # 减速归零时间 s
    num = int(zero_time * hertz)  # 平滑减速指令次数
    linear_reduce = linear / num  # 线速度次减量
    angular_reduce = angular / num  # 角速度次减量
    print(f"reduce num: {num}, lr: {linear_reduce}, ar: {angular_reduce}")

    twist_reduce = Twist()
    for n in range(num):
        # 线速度计算
        if linear > 0:
            l = linear - linear_reduce * (n + 1)
            if l < 0:
                l = 0
            twist_reduce.linear = l
        else:
            l = linear + linear_reduce * (n + 1)
            if l > 0:
                l = 0
            twist_reduce.linear = l

        # 角速度计算
        if angular > 0:
            a = angular - angular_reduce * (n + 1)
            if a < 0:
                a = 0
            twist_reduce.angular = a
        else:
            a = angular + angular_reduce * (n + 1)
            if a > 0:
                a = 0
            twist_reduce.angular = a

        print(f"twist_reduce l: {twist_reduce.linear}, a: {twist_reduce.angular}")
        twist_result, ok, msg = await robot.twist_req(twist_reduce, NO_PRINT, NO_PRINT)
        if ok:
            print("速度控制请求成功")
        else:
            print(f"速度控制请求失败, msg: {msg}")

        await asyncio.sleep(delay)

    # 保险起见再发一次零速度
    twist_zero = Twist()
    twist_result, ok, msg = await robot.twist_req(twist_zero, NO_PRINT, NO_PRINT)
    if ok:
        print("速度控制请求成功")
    else:
        print(f"速度控制请求失败, msg: {msg}")

    input("输入回车退出程序\n")


if __name__ == "__main__":
    asyncio.run(main())
