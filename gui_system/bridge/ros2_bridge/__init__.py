"""GUI 백엔드를 위한 Python ROS2 Bridge.

이 패키지는 ROS2 노드(`rclpy`)로 실행되며 gRPC(`grpc.aio`) 또는 순수
WebSocket 서버를 노출한다. ROS2 토픽/서비스와 Go 백엔드 사이를
매개해, 백엔드가 rclpy/CGo에 직접 의존하지 않게 한다. 설계 §3 참고.
"""
