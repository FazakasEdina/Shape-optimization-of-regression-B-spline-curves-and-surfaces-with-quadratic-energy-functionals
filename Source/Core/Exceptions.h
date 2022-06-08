#pragma once

#include <iostream>
#include <string>
#include <QMessageBox>


namespace cagd
{
    class Exception
    {
        friend std::ostream& operator <<(std::ostream& lhs, const Exception& rhs);

    protected:
        std::string _reason;

    public:
        Exception(const std::string &reason): _reason(reason)
        {
        }

        void showReason() const
                {
                    QMessageBox::critical(nullptr, "Exception has occured...", QString::fromStdString(_reason));
                }

    };

    inline std::ostream& operator <<(std::ostream& lhs, const Exception& rhs)
    {
        return lhs << rhs._reason;
    }
}
