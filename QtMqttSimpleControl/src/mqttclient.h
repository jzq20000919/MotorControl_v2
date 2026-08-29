#pragma once

#include <QByteArray>
#include <QObject>
#include <QString>
#include <QMqttClient>

class MqttClient final : public QObject
{
    Q_OBJECT

public:
    explicit MqttClient(QObject *parent = nullptr);

    void connectToBroker(const QString &host, quint16 port,
                         const QString &clientId);
    void disconnectFromBroker();
    bool isConnected() const;
    bool publishQos1(const QString &topic, const QByteArray &payload);
    bool subscribeQos1(const QString &topic);

signals:
    void connected();
    void disconnected();
    void messageReceived(const QString &topic, const QByteArray &payload);
    void errorOccurred(const QString &message);

private:
    static QString errorMessage(QMqttClient::ClientError error);

    QMqttClient client_;
};
