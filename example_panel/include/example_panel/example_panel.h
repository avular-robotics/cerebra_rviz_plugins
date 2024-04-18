// Copyright (C) 2024 Avular B.V. - All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file. If not, please write to: legal@avular.com

#pragma once

#include <QtWidgets>
#include <QPushButton>
#include <QLineEdit>

#include "rviz_common/panel.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "rviz_common/properties/property_tree_model.hpp"
#include "rviz_common/properties/property_tree_with_help.hpp"
#include "rviz_common/properties/string_property.hpp"

namespace cerebra_rviz_plugins
{
    class ExamplePanel : public rviz_common::Panel
    {
        Q_OBJECT

        public:
            explicit ExamplePanel(QWidget *parent = 0);
            virtual ~ExamplePanel() = default;

            void onInitialize() override;

            /// Load and save configuration data
            void load(const rviz_common::Config &config) override;
            void save(rviz_common::Config config) const override;
        private Q_SLOTS:
            void initializeTopics();
            void onButtonPressed();
            void onTopicMessage(std_msgs::msg::String::SharedPtr message);
        private:
            rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _publisher;
            rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _subscription;

            QLabel *_topic_display{nullptr};
            QLineEdit *_field{nullptr};
            QPushButton *_button{nullptr};

            QVBoxLayout *_main_layout{nullptr};
            QHBoxLayout *_publish_layout{nullptr};

            rviz_common::properties::PropertyTreeModel *_property_model;
            rviz_common::properties::PropertyTreeWithHelp *_property_widget;
            rviz_common::properties::StringProperty *_input_topic;
            rviz_common::properties::StringProperty *_output_topic;
    };
}
