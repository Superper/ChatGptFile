// logmanager.cpp
#include "LogManager.h"
#include<QDateTime>

LogManager *LogManager::instance()
{
    static LogManager s_logManager;
    return &s_logManager;
}

void LogManager::addText(const std::string &text)
{
    QString str = QString::fromStdString(text);
    if (m_pLogWidget && m_pLogWidget->isVisible())
    {
        emit logText(str);
    }
}

void LogManager::SetCheckBoxState1(bool state)
{
    m_pCheckBox_1->setChecked(state);
}

void LogManager::SetCheckBoxState2(bool state)
{
    m_pCheckBox_2->setChecked(state);
}

void LogManager::SetCheckBoxState3(bool state)
{
    m_pCheckBox_3->setChecked(state);
}

void LogManager::SetCheckBoxState4(bool state)
{
    m_pCheckBox_4->setChecked(state);
}

void LogManager::onLogText(const QString &text)
{
    QMutexLocker locker(&m_mutex);
    if (m_pLogWidget)
    {
        m_pLogWidget->appendPlainText(QDateTime::currentDateTime().toString("hh:mm:ss") + " " +text);
    }
}

LogManager::LogManager(QObject *parent) : QObject(parent), m_pLogWidget(nullptr)
{
    // 连接信号/槽
    connect(this, SIGNAL(logText(const QString &)), this, SLOT(onLogText(const QString &)));
}

LogManager::~LogManager()
{
    if (m_pLogWidget)
    {
        delete m_pLogWidget;
    }
}
