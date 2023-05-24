// logmanager.h
#ifndef LOGMANAGER_H
#define LOGMANAGER_H
#include <QMutex>
#include <QObject>
#include <QPlainTextEdit>
#include <QCheckBox>

class LogManager : public QObject
{
    Q_OBJECT
  public:
    static LogManager *instance();

    void addText(const std::string &text);

    QPlainTextEdit *m_pLogWidget;
    QCheckBox *m_pCheckBox_1;
    QCheckBox *m_pCheckBox_2;
    QCheckBox *m_pCheckBox_3;
    QCheckBox *m_pCheckBox_4;
    void SetCheckBoxState1(bool state);
    void SetCheckBoxState2(bool state);
    void SetCheckBoxState3(bool state);
    void SetCheckBoxState4(bool state);
  signals:
    void logText(const QString &text);

  private slots:
    void onLogText(const QString &text);

  private:
    explicit LogManager(QObject *parent = nullptr);
    ~LogManager();
    LogManager(const LogManager &) = delete;
    LogManager &operator=(const LogManager &) = delete;

    QMutex m_mutex;
};
#endif // LOGMANAGER_H