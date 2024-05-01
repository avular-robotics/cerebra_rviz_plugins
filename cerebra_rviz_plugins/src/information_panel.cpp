// Copyright (C) 2024 Avular B.V. - All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file. If not, please write to: legal@avular.com

#include <pluginlib/class_list_macros.hpp>
#include "information_panel/information_panel.h"
#include "rviz_common/display_context.hpp"
#include "rviz_common/properties/property.hpp"
#include "rviz_common/properties/property_tree_widget.hpp"

namespace cerebra_rviz_plugins
{
InformationPanel::InformationPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  _status_display = new QLabel;
  _battery_display = new QLabel;

  _main_layout = new QGridLayout;
  _main_layout->setColumnStretch(1, 1);
  _main_layout->addWidget(new QLabel("Status:"), 0, 0);
  _main_layout->addWidget(_status_display, 0, 1);
  _main_layout->addWidget(new QLabel("Battery Information:"), 1, 0);
  _main_layout->addWidget(_battery_display);
  _main_layout->setContentsMargins(10, 10, 10, 10);

  _root_property = new rviz_common::properties::Property("Panel properties");
  _root_property->setReadOnly(true);
  // _property_model = new rviz_common::properties::PropertyTreeModel(root_property);
  // _property_widget = new rviz_common::properties::PropertyTreeWithHelp;
  // _property_widget->getTree()->setModel(_property_model);
  // _main_layout->addWidget(_property_widget);

  _status_topic = new rviz_common::properties::StringProperty(
    "Status Topic", "/autopilot/behaviortree_executor/status",
    "The topic for retrieving the robot's status message",
    _root_property, SLOT(initializeTopics()), this);
  _battery_topic = new rviz_common::properties::StringProperty(
    "Battery Topic", "/robot/battery/info",
    "The topic for retrieving the robot's battery information",
    _root_property, SLOT(initializeTopics()), this);

  setLayout(_main_layout);
}

void InformationPanel::onInitialize()
{
  initializeTopics();
}

void InformationPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  // _property_widget->save(config);
  _root_property->save(config);
}

void InformationPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
  // _property_widget->save(config);
  _root_property->load(config);
}

void InformationPanel::initializeTopics()
{
  if (auto ros_node_abstraction = getDisplayContext()->getRosNodeAbstraction().lock()) {
    auto node = ros_node_abstraction->get_raw_node();

    _statusSubscription = node->create_subscription<autonomy_msgs::msg::StatusString>(
      _status_topic->getStdString(), 1,
      std::bind(&InformationPanel::onStatusMessage, this, std::placeholders::_1));

    _batterySubscription = node->create_subscription<origin_msgs::msg::BatteryInfo>(
      _battery_topic->getStdString(), 1,
      std::bind(&InformationPanel::onBatteryMessage, this, std::placeholders::_1));
  }
}

void InformationPanel::onStatusMessage(autonomy_msgs::msg::StatusString::SharedPtr message)
{
  _status_display->setText(QString::fromStdString(message->status));
}

void InformationPanel::onBatteryMessage(origin_msgs::msg::BatteryInfo::SharedPtr message)
{
  QString display_message;
  QTextStream display_message_stream{&display_message};
  display_message_stream << "Voltage: " << message->voltage << "V; ";
  display_message_stream << "State of charge: " << message->state_of_charge << "%";
  _battery_display->setText(display_message);
}
}  // namespace cerebra_rviz_plugins
PLUGINLIB_EXPORT_CLASS(cerebra_rviz_plugins::InformationPanel, rviz_common::Panel)
