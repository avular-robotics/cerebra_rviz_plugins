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
  _status_display->setText("<i>No data received</i>");
  _status_display->setAlignment(Qt::AlignLeft | Qt::AlignTop);
  _battery_display = new QLabel;
  _battery_display->setText("<i>No data received</i>");
  _battery_display->setAlignment(Qt::AlignLeft | Qt::AlignTop);

  _status_stale = new QLabel{QString("<font color=\"darkorange\"><b>⚠️</b></font>")};
  _status_stale->setToolTip("No status message received in the last 3 seconds");
  _status_stale->setAlignment(Qt::AlignLeft | Qt::AlignTop);
  _status_stale->hide();
  _battery_stale = new QLabel{QString("<font color=\"darkorange\"><b>⚠️</b></font>")};
  _battery_stale->setToolTip("No battery information received in the last 3 seconds");
  _battery_stale->setAlignment(Qt::AlignLeft | Qt::AlignTop);
  _battery_stale->hide();

  auto status_prefix = new QLabel("Status:", this);
  status_prefix->setAlignment(Qt::AlignRight | Qt::AlignTop);
  auto battery_prefix = new QLabel("Battery Information:", this);
  battery_prefix->setAlignment(Qt::AlignRight | Qt::AlignTop);

  _main_layout = new QGridLayout;
  _main_layout->setColumnStretch(1, 1);
  _main_layout->addWidget(status_prefix, 0, 0);
  _main_layout->addWidget(_status_display, 0, 1);
  _main_layout->addWidget(_status_stale, 0, 2);

  _main_layout->addWidget(battery_prefix, 1, 0);
  _main_layout->addWidget(_battery_display, 1, 1);
  _main_layout->addWidget(_battery_stale, 1, 2);
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

  _status_stale_timer = new QTimer(this);
  _status_stale_timer->setSingleShot(true);
  connect(_status_stale_timer, SIGNAL(timeout()), this, SLOT(onStaleStatus()));
  _status_stale_timer->start(3000);

  _battery_stale_timer = new QTimer(this);
  _battery_stale_timer->setSingleShot(true);
  connect(_battery_stale_timer, SIGNAL(timeout()), this, SLOT(onStaleBattery()));
  _battery_stale_timer->start(3000);
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
  _status_stale->hide();
  _status_stale_timer->start(3000);
}

void InformationPanel::onBatteryMessage(origin_msgs::msg::BatteryInfo::SharedPtr message)
{
  _battery_display->setText(
    QString("Voltage: %1V | State of charge: %2%")
    .arg(message->voltage, 0, 'f', 2)
    .arg(message->state_of_charge));
  _battery_stale->hide();
  _battery_stale_timer->start(3000);
}

void InformationPanel::onStaleStatus()
{
  _status_stale->show();
}

void InformationPanel::onStaleBattery()
{
  _battery_stale->show();
}
}  // namespace cerebra_rviz_plugins
PLUGINLIB_EXPORT_CLASS(cerebra_rviz_plugins::InformationPanel, rviz_common::Panel)
