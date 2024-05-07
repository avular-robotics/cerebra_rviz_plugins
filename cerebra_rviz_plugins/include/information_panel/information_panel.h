// Copyright (C) 2024 Avular B.V. - All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file. If not, please write to: legal@avular.com

#pragma once

#include <QtWidgets>
#include <QLabel>

#include "rviz_common/panel.hpp"
#include "rclcpp/rclcpp.hpp"
#include "autonomy_msgs/msg/status_string.hpp"
#include "origin_msgs/msg/battery_info.hpp"
#include "rviz_common/properties/property_tree_model.hpp"
#include "rviz_common/properties/property_tree_with_help.hpp"
#include "rviz_common/properties/string_property.hpp"

namespace cerebra_rviz_plugins
{
  class InformationPanel: public rviz_common::Panel
  {
    Q_OBJECT

public:
    explicit InformationPanel(QWidget * parent = 0);
    virtual ~InformationPanel() = default;

    void onInitialize() override;

    /// Load and save configuration data
    void load(const rviz_common::Config & config) override;
    void save(rviz_common::Config config) const override;

private Q_SLOTS:
    void initializeTopics();
    void onStatusMessage(autonomy_msgs::msg::StatusString::SharedPtr message);
    void onBatteryMessage(origin_msgs::msg::BatteryInfo::SharedPtr message);

    void onStaleTimer();

private:
    rclcpp::Subscription < autonomy_msgs::msg::StatusString > ::SharedPtr _statusSubscription;
    rclcpp::Subscription < origin_msgs::msg::BatteryInfo > ::SharedPtr _batterySubscription;

    bool _status_received {false};
    bool _battery_received {false};

    QLabel * _status_display {nullptr};
    QLabel * _battery_display {nullptr};

    QLabel * _status_stale {nullptr};
    QLabel * _battery_stale {nullptr};

    QTimer * _stale_timer {nullptr};

    QGridLayout * _main_layout {nullptr};

    // rviz_common::properties::PropertyTreeModel * _property_model;
    // rviz_common::properties::PropertyTreeWithHelp * _property_widget;
    rviz_common::properties::Property * _root_property;
    rviz_common::properties::StringProperty * _status_topic;
    rviz_common::properties::StringProperty * _battery_topic;
  };
}
