#!/usr/bin/env python3

from pathlib import Path
import unittest


PACKAGE_DIR = Path(__file__).resolve().parents[1]
PUB_SOURCE = (PACKAGE_DIR / "src" / "image_pub.cpp").read_text()
SUB_SOURCE = (PACKAGE_DIR / "src" / "image_sub.cpp").read_text()
PUB_HEADER = (PACKAGE_DIR / "include" / "image_test" / "image_pub.hpp").read_text()
SUB_HEADER = (PACKAGE_DIR / "include" / "image_test" / "image_sub.hpp").read_text()


def function_body(source: str, signature: str) -> str:
    start = source.index(signature)
    brace = source.index("{", start)
    depth = 0
    for index in range(brace, len(source)):
        char = source[index]
        if char == "{":
            depth += 1
        elif char == "}":
            depth -= 1
            if depth == 0:
                return source[brace:index + 1]
    raise AssertionError(f"Could not parse function body for {signature}")


def block_from(source: str, marker: str) -> str:
    start = source.index(marker)
    brace = source.index("{", start)
    depth = 0
    for index in range(brace, len(source)):
        char = source[index]
        if char == "{":
            depth += 1
        elif char == "}":
            depth -= 1
            if depth == 0:
                return source[brace:index + 1]
    raise AssertionError(f"Could not parse block for {marker}")


class TimestampPositionTest(unittest.TestCase):
    def assert_before(self, body: str, first: str, second: str) -> None:
        self.assertIn(first, body)
        self.assertIn(second, body)
        self.assertLess(body.index(first), body.index(second))

    def test_modes_1_2_3_stamp_after_source_cvmat_is_ready(self) -> None:
        body = function_body(PUB_SOURCE, "void image_pub::publish_image1()")

        self.assert_before(
            body,
            "if (image.empty())",
            "const auto image_ready_time = this->now();",
        )
        self.assert_before(
            body,
            "const auto image_ready_time = this->now();",
            "switch (this->mode)",
        )
        self.assert_before(
            body,
            "const auto image_ready_steady = std::chrono::steady_clock::now();",
            "switch (this->mode)",
        )
        self.assertIn("image_msg_->header.stamp  = image_ready_time;", body)
        self.assertIn("sender->send(image, image_ready_steady);", body)
        self.assertIn("ImagePack(std::move(image), image_ready_time)", body)

    def test_mode_4_can_compare_direct_generation_and_copy_path(self) -> None:
        body = function_body(PUB_SOURCE, "void image_pub::publish_image2()")
        direct_body = block_from(body, "if (generate_in_transport_buffer_)")

        self.assertIn("if (generate_in_transport_buffer_)", body)
        self.assert_before(
            body,
            "if (!generate_in_transport_buffer_)",
            "auto loanedMsg = loaned_img_pub_->borrow_loaned_message();",
        )
        self.assert_before(
            body,
            "image_ready_time = now();",
            "auto loanedMsg = loaned_img_pub_->borrow_loaned_message();",
        )
        self.assert_before(
            body,
            "auto loanedMsg = loaned_img_pub_->borrow_loaned_message();",
            "cv::Mat wrapper = prepare_image8m_payload(msg);",
        )
        self.assert_before(
            direct_body,
            "wrapper.setTo(cv::Scalar(0, 0, 0));",
            "image_ready_time = now();",
        )
        self.assertEqual(body.count("msg.header.stamp = image_ready_time;"), 2)
        self.assertEqual(body.count("wrapper.setTo(cv::Scalar(0, 0, 0));"), 2)
        self.assertEqual(body.count("image.copyTo(wrapper);"), 2)
        self.assert_before(
            body,
            "if (!generate_in_transport_buffer_)",
            "image.copyTo(wrapper);",
        )

    def test_mode_5_can_compare_direct_generation_and_copy_path(self) -> None:
        body = function_body(PUB_SOURCE, "void image_pub::publish_image_iceoryx()")
        direct_body = block_from(body, "if (generate_in_transport_buffer_)")
        copy_body = block_from(body, "if (!generate_in_transport_buffer_)")

        self.assertIn("if (generate_in_transport_buffer_)", body)
        self.assertIn("image.copyTo(wrapper);", body)
        self.assert_before(
            body,
            "if (!generate_in_transport_buffer_)",
            "auto result = iceoryx_pub_->loan",
        )
        self.assertIn("image_ready_steady = std::chrono::steady_clock::now();", copy_body)
        self.assert_before(
            body,
            "auto result = iceoryx_pub_->loan",
            "cv::Mat wrapper(kImageHeight, kImageWidth, CV_8UC3, img_data);",
        )
        self.assert_before(
            body,
            "cv::Mat wrapper(kImageHeight, kImageWidth, CV_8UC3, img_data);",
            "wrapper.setTo(cv::Scalar(0, 0, 0));",
        )
        self.assert_before(
            direct_body,
            "wrapper.setTo(cv::Scalar(0, 0, 0));",
            "image_ready_steady = std::chrono::steady_clock::now();",
        )
        self.assertIn("image_ready_steady.time_since_epoch()", body)

    def test_transport_buffer_generation_is_launch_configurable(self) -> None:
        body = function_body(PUB_SOURCE, "image_pub::image_pub")
        launch_source = (PACKAGE_DIR / "launch" / "image_test.launch.py").read_text()

        self.assertIn('declare_parameter("generate_in_transport_buffer", true)', body)
        self.assertIn("generate_in_transport_buffer_", PUB_SOURCE)
        self.assertIn("DeclareLaunchArgument(", launch_source)
        self.assertIn("name='generate_in_transport_buffer'", launch_source)
        self.assertIn("'generate_in_transport_buffer': LaunchConfiguration(", launch_source)

    def test_mode_6_uses_rclcpp_unique_ptr_intra_process_path(self) -> None:
        pub_constructor = function_body(PUB_SOURCE, "image_pub::image_pub")
        sub_constructor = function_body(SUB_SOURCE, "image_sub::image_sub")
        pub_body = function_body(PUB_SOURCE, "void image_pub::publish_image_unique()")
        sub_body = function_body(SUB_SOURCE, "void image_sub::uniqueImageCallback")

        self.assertIn("case 6:", pub_constructor)
        self.assertIn("case 6:", sub_constructor)
        self.assertIn("create_publisher<sensor_msgs::msg::Image>", pub_constructor)
        self.assertIn('"image_raw_unique"', pub_constructor)
        self.assertIn(
            "std::make_unique<sensor_msgs::msg::Image>()",
            pub_body,
        )
        self.assertIn("raw_img_pub_->publish(std::move(msg));", pub_body)
        self.assertIn(
            "sensor_msgs::msg::Image::UniquePtr img_msg",
            SUB_SOURCE,
        )
        self.assertIn("uniqueImageCallback", sub_constructor)
        self.assertIn("cv::Mat received_image", sub_body)
        self.assertIn("msg->data.resize(kImagePayloadSize);", pub_body)
        self.assertIn("msg->data.data()", pub_body)
        self.assertIn("if (!generate_in_transport_buffer_)", pub_body)
        self.assertIn("if (generate_in_transport_buffer_)", pub_body)
        self.assertIn("image.copyTo(wrapper);", pub_body)
        self.assert_before(
            pub_body,
            "if (!generate_in_transport_buffer_)",
            "auto msg = std::make_unique<sensor_msgs::msg::Image>();",
        )
        self.assertIn("img_msg->step", sub_body)

    def test_mode_7_uses_autoaim_shared_memory_ring_path(self) -> None:
        pub_constructor = function_body(PUB_SOURCE, "image_pub::image_pub")
        sub_constructor = function_body(SUB_SOURCE, "image_sub::image_sub")
        pub_body = function_body(PUB_SOURCE, "void image_pub::publish_image_autoaim_shm()")
        sub_body = function_body(SUB_SOURCE, "void image_sub::startAutoAimShmReceiver(bool copy_image)")
        launch_source = (PACKAGE_DIR / "launch" / "image_test.launch.py").read_text()

        self.assertIn("case 7:", pub_constructor)
        self.assertIn("case 7:", sub_constructor)
        self.assertIn(
            '#include "autoaim_shm_image_transport/autoaim_shm_image_transport.hpp"',
            PUB_HEADER,
        )
        self.assertIn(
            '#include "autoaim_shm_image_transport/autoaim_shm_image_transport.hpp"',
            SUB_HEADER,
        )
        self.assertNotIn(
            '#include "image_test/autoaim_shm_image_transport.hpp"',
            PUB_HEADER,
        )
        self.assertNotIn(
            '#include "image_test/autoaim_shm_image_transport.hpp"',
            SUB_HEADER,
        )
        self.assertIn("autoaim_shm_publisher_", PUB_SOURCE)
        self.assertIn("autoaim_shm_subscriber_", SUB_SOURCE)
        self.assertIn("publish_image_autoaim_shm", pub_constructor)
        self.assertIn("startAutoAimShmReceiver(copy_image)", sub_constructor)
        self.assertIn("autoaim_shm_name", launch_source)
        self.assertIn("'autoaim_shm_name': LaunchConfiguration('autoaim_shm_name')", launch_source)
        self.assertIn("mode 7", launch_source)

        self.assertIn(
            "autoaim_shm_image_transport::autoaim_shm_steady_time_ns()",
            pub_body,
        )
        self.assertIn("if (generate_in_transport_buffer_)", pub_body)
        self.assertIn("auto frame = autoaim_shm_publisher_->borrow_frame", pub_body)
        self.assertIn("frame.image().setTo(cv::Scalar(0, 0, 0));", pub_body)
        self.assertIn("frame.commit(image_ready_steady_ns);", pub_body)
        self.assert_before(
            pub_body,
            "if (generate_in_transport_buffer_)",
            "if (image.empty())",
        )
        direct_body = block_from(pub_body, "if (generate_in_transport_buffer_)")
        self.assert_before(
            direct_body,
            "frame.image().setTo(cv::Scalar(0, 0, 0));",
            "const auto image_ready_steady_ns",
        )
        self.assert_before(
            direct_body,
            "const auto image_ready_steady_ns",
            "frame.commit(image_ready_steady_ns);",
        )
        self.assert_before(
            pub_body,
            "if (image.empty())",
            "autoaim_shm_publisher_->publish(image, image_ready_steady_ns);",
        )
        self.assert_before(
            pub_body,
            "const auto image_ready_steady_ns",
            "autoaim_shm_publisher_->publish(image, image_ready_steady_ns);",
        )

        self.assertIn("wait_for_frame(copy_image)", sub_body)
        self.assertIn("frame.publish_time_ns", sub_body)
        self.assertIn("autoaim_shm_image_transport::autoaim_shm_steady_time_ns()", sub_body)
        self.assertIn("RCLCPP_INFO_STREAM", sub_body)

    def test_shm_video_latency_uses_frame_start_timestamp(self) -> None:
        body = function_body(SUB_SOURCE, "void image_sub::startShmVideoReceiver(bool copy_image)")

        self.assertIn("now_time - receivedFrame.time_stamp", body)
        self.assertNotIn("now_time - receivedFrame.write_time", body)


if __name__ == "__main__":
    unittest.main()
