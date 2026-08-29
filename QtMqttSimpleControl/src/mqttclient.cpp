#include "mqttclient.h"

#include <QMqttTopicFilter>
#include <QMqttTopicName>

namespace {
constexpr quint16 kKeepAliveSeconds = 20U;
constexpr quint8 kQosAtLeastOnce = 1U;
}

MqttClient::MqttClient(QObject *parent)
    : QObject(parent)
    , client_(this)
{
    client_.setProtocolVersion(QMqttClient::MQTT_3_1_1);
    client_.setCleanSession(true);
    client_.setKeepAlive(kKeepAliveSeconds);
    client_.setAutoKeepAlive(true);

    connect(&client_, &QMqttClient::connected,
            this, &MqttClient::connected);
    connect(&client_, &QMqttClient::disconnected,
            this, &MqttClient::disconnected);
    connect(&client_, &QMqttClient::messageReceived, this,
            [this](const QByteArray &payload,
                   const QMqttTopicName &topic) {
                emit messageReceived(topic.name(), payload);
            });
    connect(&client_, &QMqttClient::errorChanged, this,
            [this](QMqttClient::ClientError error) {
                if (error != QMqttClient::NoError) {
                    emit errorOccurred(errorMessage(error));
                }
            });
}

void MqttClient::connectToBroker(const QString &host, quint16 port,
                                 const QString &clientId)
{
    if (client_.state() != QMqttClient::Disconnected) {
        emit errorOccurred(tr("MQTT client is not disconnected"));
        return;
    }

    client_.setHostname(host);
    client_.setPort(port);
    client_.setClientId(clientId);
    client_.connectToHost();
}

void MqttClient::disconnectFromBroker()
{
    if (client_.state() != QMqttClient::Disconnected) {
        client_.disconnectFromHost();
    }
}

bool MqttClient::isConnected() const
{
    return client_.state() == QMqttClient::Connected;
}

bool MqttClient::publishQos1(const QString &topic,
                             const QByteArray &payload)
{
    const QMqttTopicName topicName(topic);
    if (!isConnected() || !topicName.isValid()) {
        return false;
    }

    return client_.publish(topicName, payload, kQosAtLeastOnce, false) != -1;
}

bool MqttClient::subscribeQos1(const QString &topic)
{
    const QMqttTopicFilter topicFilter(topic);
    if (!isConnected() || !topicFilter.isValid()) {
        return false;
    }

    return client_.subscribe(topicFilter, kQosAtLeastOnce) != nullptr;
}

QString MqttClient::errorMessage(QMqttClient::ClientError error)
{
    switch (error) {
    case QMqttClient::NoError:
        return tr("No MQTT error");
    case QMqttClient::InvalidProtocolVersion:
        return tr("The broker rejected MQTT 3.1.1");
    case QMqttClient::IdRejected:
        return tr("The broker rejected the client ID");
    case QMqttClient::ServerUnavailable:
        return tr("The MQTT broker is unavailable");
    case QMqttClient::BadUsernameOrPassword:
        return tr("Invalid MQTT username or password");
    case QMqttClient::NotAuthorized:
        return tr("The MQTT connection is not authorized");
    case QMqttClient::TransportInvalid:
        return tr("The MQTT transport is invalid");
    case QMqttClient::ProtocolViolation:
        return tr("The broker reported an MQTT protocol violation");
    case QMqttClient::UnknownError:
        return tr("Unknown MQTT error");
    case QMqttClient::Mqtt5SpecificError:
        return tr("MQTT 5 specific error");
    }
    return tr("MQTT error code %1").arg(static_cast<int>(error));
}
