#include "stdafx.h"
#include "displaywidget.h"


DisplayWidget::DisplayWidget(QWidget* parent) : QWidget(parent)
{
    m_pWidget = new QWidget(this);
    m_pWidget->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);

    createUltrasonicButtonGroupBox();
    createStopLineButtonGroupBox();

    createGridGroupBox();

    m_btSendValueTrue = new QPushButton(this);
    m_btSendValueTrue->setText("Enable keyboard arrow control");
    m_btSendValueTrue->setFixedSize(200, 50);

    m_btSendValueFalse = new QPushButton(this);
    m_btSendValueFalse->setText("Disable keyboard arrow control");
    m_btSendValueFalse->setFixedSize(200, 50);

    m_mainLayout = new QVBoxLayout();
    m_mainLayout->addWidget(m_btSendValueFalse, 0, Qt::AlignCenter);
    m_mainLayout->addWidget(m_btSendValueTrue, 0, Qt::AlignCenter);
    m_mainLayout->addWidget(horizontalGroupBox);
    m_mainLayout->addWidget(gridGroupBox);


    setLayout(m_mainLayout);

    connect(m_btSendValueFalse, SIGNAL(clicked()), this,
            SLOT(sendValueFalse()));
    connect(m_btSendValueTrue, SIGNAL(clicked()), this, SLOT(sendValueTrue()));

    setFocusPolicy(Qt::StrongFocus);
}

void DisplayWidget::createUltrasonicButtonGroupBox()
{
    horizontalGroupBox = new QGroupBox(tr("Enable Ultrasonic"));
    QHBoxLayout *layout = new QHBoxLayout;

    m_btEnableUltrasonic = new QPushButton(tr("Enable Ultrasonic"));
    m_btDisableUltrasonic = new QPushButton(tr("Disable Ultrasonic"));
    layout->addWidget(m_btEnableUltrasonic);
    layout->addWidget(m_btDisableUltrasonic);

    horizontalGroupBox->setLayout(layout);
}

void DisplayWidget::createStopLineButtonGroupBox()
{
    horizontalGroupBox = new QGroupBox(tr("Enable StopLine"));
    QHBoxLayout *layout = new QHBoxLayout;

    m_btEnableUltrasonic = new QPushButton(tr("Enable StopLine"));
    m_btDisableUltrasonic = new QPushButton(tr("Disable StopLine"));
    layout->addWidget(m_btEnableUltrasonic);
    layout->addWidget(m_btDisableUltrasonic);

    horizontalGroupBox->setLayout(layout);
}

void DisplayWidget::createGridGroupBox()
{
    gridGroupBox = new QGroupBox(tr("Grid layout"));
    QGridLayout *layout = new QGridLayout;

    smallEditor = new QTextEdit;
    smallEditor->setPlainText(tr("This widget takes up about two thirds of the "
                                     "grid layout."));
    layout->addWidget(smallEditor, 0, 2, 4, 1);

    layout->setColumnStretch(1, 10);
    layout->setColumnStretch(2, 20);
    gridGroupBox->setLayout(layout);
}


void DisplayWidget::keyPressEvent(QKeyEvent* event) {
    if (!event->isAutoRepeat()) {
        emit keyReceived((int)event->key());

        int key = event->key();
        if (key == 16777235) {
            // UP
            emit sendpressUp();
        } else if (key == 16777234) {
            // LEFT
            emit sendpressLeft();
        } else if (key == 16777236) {
            // Right
            emit sendpressRight();
        } else if (key == 16777237) {
            // DOWN
            emit sendpressDown();
        }
    }
}

void DisplayWidget::keyReleaseEvent(QKeyEvent* event) {
    if (!event->isAutoRepeat()) {
        //        emit keyReceived((int)event->key());

        int key = event->key();
        if (key == 16777235) {
            // UP
            emit sendreleaseUp();
        } else if (key == 16777234) {
            // LEFT
            emit sendreleaseLeft();
        } else if (key == 16777236) {
            // Right
            emit sendreleaseRight();
        } else if (key == 16777237) {
            // DOWN
            emit sendreleaseDown();
        }
    }
}

void DisplayWidget::mousePressEvent(QMouseEvent* event) {
    // printf("\nMouse in board");
    setFocus();
}
void DisplayWidget::focusOutEvent(QFocusEvent* e) {
    if (e->reason() == Qt::MouseFocusReason) {
        // Resize the geometry -> resize(bigWidth,bigHeight);
    }
    emit sendoutFocusWidget();
}

