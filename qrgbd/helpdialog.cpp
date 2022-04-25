#include "helpdialog.h"
#include "ui_helpdialog.h"

HelpDialog::HelpDialog(QWidget *parent) : QDialog(parent),
                                          ui(new Ui::HelpDialog)
{
    ui->setupUi(this);
    this->setWindowTitle("Help Document");
    connect(ui->btn_ok, &QPushButton::clicked, this, [=]() {
        this->close();
    });
}

HelpDialog::~HelpDialog()
{
    delete ui;
}
