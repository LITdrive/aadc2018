#ifndef DISPWIDGET_VALUEGENERATOR_FILTER
#define DISPWIDGET_VALUEGENERATOR_FILTER


#include "stdafx.h"

/*!
This class it the QWidget for the prototyp of the driver filter
*/
class DisplayWidget: public QWidget
{
    Q_OBJECT

public:
    /*! constructor for the widget
    * \param parent the parent widget
    */
    DisplayWidget(QWidget* parent);

    /*! Destructor. */
    ~DisplayWidget() {};

    enum { NumGridRows = 3,
           numModeButtons = 4 };


    QPushButton *m_btSendValueTrue;
    QPushButton *m_btSendValueFalse;

    QGroupBox *horizontalGroupBox;
    QPushButton *m_btEnableUltrasonic;
    QPushButton *m_btDisableUltrasonic;


    QGroupBox *gridGroupBox;
    QTextEdit *smallEditor;


signals:
    void keyReceived(int value);
    void sendoutFocusWidget();
    void sendpressUp();
    void sendpressDown();
    void sendpressLeft();
    void sendpressRight();
    void sendreleaseUp();
    void sendreleaseDown();
    void sendreleaseLeft();
    void sendreleaseRight();

private:
    /*! the main widget */
    QWidget* m_pWidget;

    /*! the main layout for the widget*/
    QVBoxLayout *m_mainLayout;

protected:
   void keyPressEvent(QKeyEvent *event);
   void keyReleaseEvent(QKeyEvent *event);
   void mousePressEvent(QMouseEvent *event);
   void focusOutEvent(QFocusEvent *e);

   void createUltrasonicButtonGroupBox();
   void createStopLineButtonGroupBox();
   void createGridGroupBox();
};

#endif
