// Copyright (C) 2024 Avular B.V. - All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file. If not, please write to: legal@avular.com

#include <pluginlib/class_list_macros.hpp>
#include "example_panel/example_panel.h"
#include "rviz_common/display_context.hpp"
#include "rviz_common/properties/property.hpp"
#include "rviz_common/properties/property_tree_widget.hpp"

namespace cerebra_rviz_plugins
{
    ExamplePanel::ExamplePanel(QWidget *parent) : rviz_common::Panel(parent)
    {
        _topic_display = new QLabel;
        _field = new QLineEdit;
        _button = new QPushButton;

        QObject::connect(_button, &QPushButton::released, this, &ExamplePanel::onButtonPressed);

        _publish_layout = new QHBoxLayout;
        _publish_layout->addWidget(_field);
        _publish_layout->addWidget(_button);
        _publish_layout->setContentsMargins(10,10,10,10);

        _main_layout = new QVBoxLayout;
        _main_layout->addWidget(_topic_display);
        _main_layout->addLayout(_publish_layout);
        _main_layout->setContentsMargins(10,10,10,10);

        auto root_property = new rviz_common::properties::Property("Panel properties");
        root_property->setReadOnly(true);
        _property_model = new rviz_common::properties::PropertyTreeModel(root_property);
        _property_widget = new rviz_common::properties::PropertyTreeWithHelp;
        _property_widget->getTree()->setModel(_property_model);
        _main_layout->addWidget(_property_widget);

        _input_topic = new rviz_common::properties::StringProperty(
            "Input Topic", "test_input_topic", "A topic to display",
            root_property, SLOT(initializeTopics()), this);
        _output_topic = new rviz_common::properties::StringProperty(
            "Output Topic", "test_output_topic", "A topic to send to",
            root_property, SLOT(initializeTopics()), this);

        setLayout(_main_layout);
    }

    void ExamplePanel::onInitialize()
    {
        initializeTopics();
    }

    void ExamplePanel::save(rviz_common::Config config) const
    {
        rviz_common::Panel::save(config);
        _property_widget->save(config);
        _property_model->getRoot()->save(config);
    }

    void ExamplePanel::load(const rviz_common::Config & config)
    {
        rviz_common::Panel::load(config);
        _property_widget->save(config);
        _property_model->getRoot()->load(config);
    }

    void ExamplePanel::initializeTopics()
    {
        if(auto ros_node_abstraction = getDisplayContext()->getRosNodeAbstraction().lock())
        {
            auto node = ros_node_abstraction->get_raw_node();
            _publisher = node->create_publisher<std_msgs::msg::String>(_output_topic->getStdString(), 1);
            _subscription = node->create_subscription<std_msgs::msg::String>(
                _input_topic->getStdString(), 1, std::bind(&ExamplePanel::onTopicMessage, this, std::placeholders::_1));
        }
    }

    void ExamplePanel::onButtonPressed()
    {
        if(_publisher)
        {
            std::string field_text = _field->text().toStdString();
            auto message = std_msgs::msg::String();
            message.data = field_text;
            _publisher->publish(message);
        }
    }

    void ExamplePanel::onTopicMessage(std_msgs::msg::String::SharedPtr message)
    {
        _topic_display->setText(QString::fromStdString(message->data));
    }
}
PLUGINLIB_EXPORT_CLASS(cerebra_rviz_plugins::ExamplePanel, rviz_common::Panel)
